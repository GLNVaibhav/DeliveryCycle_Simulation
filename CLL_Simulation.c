#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define SIM_TIME      500.0
#define ARRIVAL_MEAN  2.0
#define RATE_PLACED   0.7
#define RATE_PACKED   1.0
#define RATE_DISPATCH 0.9
#define RATE_OUTFOR   0.6
#define LOG_INTERVAL  5.0
#define P_EXPRESS     0.12
#define P_CANCEL      0.01
#define WARMUP        20.0

#define MAX_PRINT_EVENTS 200

#define FILE_DETAILED "orders_detailed_c.csv"
#define FILE_LOG      "orders_log_c.csv"
#define FILE_SUMMARY  "simulation_summary_c.txt"

double uni() {
    return (double)rand() / (double)RAND_MAX;
}

double expo(double mean) {
    if (mean <= 0.0) return 0.0;
    double u = uni();
    if (u <= 0.0) u = 1e-12;
    if (u >= 1.0) u = 1.0 - 1e-12;
    return -mean * log(1.0 - u);
}

typedef enum {
    PLACED = 0,
    PACKED = 1,
    DISPATCHED = 2,
    OUTFOR = 3,
    DELIVERED = 4
} Stage;

const char* stage_name(Stage s) {
    switch (s) {
        case PLACED:     return "PLACED";
        case PACKED:     return "PACKED";
        case DISPATCHED: return "DISPATCHED";
        case OUTFOR:     return "OUT_FOR_DELIVERY";
        case DELIVERED:  return "DELIVERED";
        default:         return "UNKNOWN";
    }
}

const char* phase_label(Stage from, Stage to) {
    if (from == PLACED && to == PACKED)       return "PACKING";
    if (from == PACKED && to == DISPATCHED)   return "DISPATCHED";
    if (from == DISPATCHED && to == OUTFOR)   return "IN_TRANSIT";
    if (from == OUTFOR && to == DELIVERED)    return "DELIVERED";
    return "STAGE_MOVE";
}

typedef struct Order {
    unsigned id;
    int express;
    Stage stage;
    double arrival_time;
    double delivered_time;
    struct Order *next;
} Order;

typedef struct {
    Order *head;
    int sz;
} Ring;

void ring_init(Ring *r) {
    r->head = NULL;
    r->sz   = 0;
}

void insert_tail(Ring *r, Order *o) {
    if (!r->head) {
        r->head = o;
        o->next = o;
        r->sz   = 1;
        return;
    }
    Order *t = r->head;
    while (t->next != r->head) t = t->next;
    t->next = o;
    o->next = r->head;
    r->sz++;
}

void insert_after_head(Ring *r, Order *o) {
    if (!r->head) {
        r->head = o;
        o->next = o;
        r->sz   = 1;
        return;
    }
    o->next = r->head->next;
    r->head->next = o;
    r->sz++;
}

Order* remove_node(Ring *r, Order *p, Order *prev) {
    if (!p || !r->head) return NULL;
    if (p == prev) {
        free(p);
        r->head = NULL;
        r->sz   = 0;
        return NULL;
    }
    prev->next = p->next;
    if (p == r->head) {
        r->head = p->next;
    }
    Order *nxt = p->next;
    free(p);
    r->sz--;
    return nxt;
}

typedef struct {
    double *times;
    int *sizes;
    int len;
    int cap;
} Samples;

void samples_init(Samples *s) {
    s->cap   = 1024;
    s->len   = 0;
    s->times = (double*)malloc(sizeof(double) * s->cap);
    s->sizes = (int*)   malloc(sizeof(int)    * s->cap);
}

void samples_push(Samples *s, double t, int sz) {
    if (s->len >= s->cap) {
        s->cap *= 2;
        s->times = (double*)realloc(s->times, sizeof(double) * s->cap);
        s->sizes = (int*)   realloc(s->sizes, sizeof(int)    * s->cap);
    }
    s->times[s->len] = t;
    s->sizes[s->len] = sz;
    s->len++;
}

int main(void) {
    srand(42);

    Ring ring;
    ring_init(&ring);

    double now          = 0.0;
    double sim_end      = SIM_TIME;
    double next_arrival = expo(ARRIVAL_MEAN);
    double next_log     = LOG_INTERVAL;
    unsigned next_id    = 1;

    Samples samp;
    samples_init(&samp);

    int total_arrived        = 0;
    int total_express        = 0;
    int total_normal         = 0;
    int delivered_count      = 0;
    int delivered_express    = 0;
    int delivered_normal     = 0;
    int cancelled_count      = 0;

    double sum_sys_time_all      = 0.0;
    double sum_sys_time_express  = 0.0;
    double sum_sys_time_normal   = 0.0;

    FILE *dout = fopen(FILE_DETAILED, "w");
    if (!dout) {
        perror("Cannot open detailed CSV");
        return 1;
    }
    fprintf(dout, "id,express,arrival,delivered,final_stage,time_in_system\n");

    Order *cur  = NULL;
    Order *prev = NULL;

    int printed_events = 0;

    printf("===== DELIVERY CYCLE SIMULATION USING CIRCULAR LINKED LIST =====\n");
    printf("Time is in abstract units. Showing up to %d key events.\n\n",
           MAX_PRINT_EVENTS);

    while (now < sim_end) {
        if (now >= next_arrival) {
            Order *o = (Order*)malloc(sizeof(Order));
            if (!o) {
                fprintf(stderr, "Memory allocation failed\n");
                break;
            }
            o->id             = next_id++;
            o->express        = (uni() < P_EXPRESS) ? 1 : 0;
            o->stage          = PLACED;
            o->arrival_time   = now;
            o->delivered_time = -1.0;
            o->next           = NULL;

            total_arrived++;
            if (o->express) total_express++; else total_normal++;

            if (o->express) insert_after_head(&ring, o);
            else           insert_tail(&ring, o);

            if (!cur) {
                cur  = ring.head;
                prev = ring.head;
                while (prev->next != cur) prev = prev->next;
            }

            if (printed_events < MAX_PRINT_EVENTS) {
                printf("[t=%7.3f] ARRIVAL    : Order %u (%s) entered at %s. Queue size = %d\n",
                       now, o->id, o->express ? "EXPRESS" : "NORMAL",
                       stage_name(o->stage), ring.sz);
                printed_events++;
            }

            next_arrival = now + expo(ARRIVAL_MEAN);
        }

        if (now >= next_log) {
            samples_push(&samp, now, ring.sz);
            next_log += LOG_INTERVAL;
        }

        if (!ring.head) {
            cur  = NULL;
            prev = NULL;
            now  = next_arrival;
            continue;
        }

        if (!cur) {
            cur  = ring.head;
            prev = ring.head;
            while (prev->next != cur) prev = prev->next;
        }

        double mean = 1.0;
        switch (cur->stage) {
            case PLACED:     mean = RATE_PLACED;   break;
            case PACKED:     mean = RATE_PACKED;   break;
            case DISPATCHED: mean = RATE_DISPATCH; break;
            case OUTFOR:     mean = RATE_OUTFOR;   break;
            default:         mean = 1.0;           break;
        }

        double service = expo(mean);
        now += service;
        if (now > sim_end) break;

        if (cur->stage != DELIVERED && cur->stage != PLACED && uni() < P_CANCEL) {
            cancelled_count++;

            if (printed_events < MAX_PRINT_EVENTS) {
                printf("[t=%7.3f] CANCELLED  : Order %u (%s) cancelled at stage %s. Queue size(before) = %d\n",
                       now, cur->id, cur->express ? "EXPRESS" : "NORMAL",
                       stage_name(cur->stage), ring.sz);
                printed_events++;
            }

            Order *nxt = remove_node(&ring, cur, prev);
            cur = nxt;
            if (!cur) {
                prev = NULL;
            }
            continue;
        }

        Stage old_stage = cur->stage;
        cur->stage = (Stage)((int)cur->stage + 1);

        if (cur->stage == DELIVERED) {
            cur->delivered_time = now;
            double t_sys = cur->delivered_time - cur->arrival_time;

            if (cur->arrival_time >= WARMUP) {
                sum_sys_time_all += t_sys;
                if (cur->express) sum_sys_time_express += t_sys;
                else              sum_sys_time_normal  += t_sys;
            }

            delivered_count++;
            if (cur->express) delivered_express++;
            else              delivered_normal++;

            fprintf(dout, "%u,%d,%.3f,%.3f,%s,%.3f\n",
                    cur->id,
                    cur->express,
                    cur->arrival_time,
                    cur->delivered_time,
                    stage_name(cur->stage),
                    t_sys);

            if (printed_events < MAX_PRINT_EVENTS) {
                printf("[t=%7.3f] DELIVERED  : Order %u (%s) completed. Time in system = %.3f, Queue size(before) = %d\n",
                       now, cur->id, cur->express ? "EXPRESS" : "NORMAL",
                       t_sys, ring.sz);
                printed_events++;
            }

            Order *nxt = remove_node(&ring, cur, prev);
            cur = nxt;
            if (!cur) {
                prev = NULL;
            }
            continue;
        }

        if (printed_events < MAX_PRINT_EVENTS) {
            const char *phase = phase_label(old_stage, cur->stage);
            printf("[t=%7.3f] %-10s: Order %u (%s) %s -> %s. Queue size = %d\n",
                   now, phase,
                   cur->id, cur->express ? "EXPRESS" : "NORMAL",
                   stage_name(old_stage), stage_name(cur->stage), ring.sz);
            printed_events++;
        }

        prev = cur;
        cur  = cur->next;
    }

    fclose(dout);

    FILE *csv = fopen(FILE_LOG, "w");
    if (!csv) {
        perror("Cannot open log CSV");
        return 1;
    }
    fprintf(csv, "time,queue_size\n");
    for (int i = 0; i < samp.len; i++) {
        fprintf(csv, "%.3f,%d\n", samp.times[i], samp.sizes[i]);
    }
    fclose(csv);

    int used_deliveries_all     = delivered_count;
    int used_deliveries_express = delivered_express;
    int used_deliveries_normal  = delivered_normal;

    double avg_all     = (used_deliveries_all     > 0) ? (sum_sys_time_all     / used_deliveries_all)     : 0.0;
    double avg_express = (used_deliveries_express > 0) ? (sum_sys_time_express / used_deliveries_express) : 0.0;
    double avg_normal  = (used_deliveries_normal  > 0) ? (sum_sys_time_normal  / used_deliveries_normal)  : 0.0;

    FILE *sumf = fopen(FILE_SUMMARY, "w");
    if (!sumf) {
        perror("Cannot open summary TXT");
        return 1;
    }

    fprintf(sumf, "=== DELIVERY CYCLE CLL SIMULATION SUMMARY ===\n\n");
    fprintf(sumf, "Simulation time          : %.2f units\n", sim_end);
    fprintf(sumf, "Mean inter-arrival time  : %.2f\n", ARRIVAL_MEAN);
    fprintf(sumf, "Stages (mean service times):\n");
    fprintf(sumf, "  PLACED     -> %.2f\n", RATE_PLACED);
    fprintf(sumf, "  PACKED     -> %.2f\n", RATE_PACKED);
    fprintf(sumf, "  DISPATCHED -> %.2f\n", RATE_DISPATCH);
    fprintf(sumf, "  OUTFOR     -> %.2f\n\n", RATE_OUTFOR);

    fprintf(sumf, "Total orders arrived     : %d\n", total_arrived);
    fprintf(sumf, "  Express orders         : %d\n", total_express);
    fprintf(sumf, "  Normal orders          : %d\n", total_normal);
    fprintf(sumf, "Total cancelled          : %d\n\n", cancelled_count);

    fprintf(sumf, "Total delivered          : %d\n", delivered_count);
    fprintf(sumf, "  Delivered express      : %d\n", delivered_express);
    fprintf(sumf, "  Delivered normal       : %d\n\n", delivered_normal);

    fprintf(sumf, "Average time in system (from arrival to delivery)\n");
    fprintf(sumf, "  Overall                : %.3f time units\n", avg_all);
    fprintf(sumf, "  Express only           : %.3f time units\n", avg_express);
    fprintf(sumf, "  Normal only            : %.3f time units\n\n", avg_normal);

    fprintf(sumf, "Queue size samples written to: %s\n", FILE_LOG);
    fprintf(sumf, "Per-order lifecycle written to: %s\n", FILE_DETAILED);

    fclose(sumf);

    printf("\n===== SIMULATION COMPLETE =====\n");
    printf("Delivered = %d, Cancelled = %d\n", delivered_count, cancelled_count);
    printf("Files generated:\n");
    printf("  %s (per-order details)\n", FILE_DETAILED);
    printf("  %s (queue size over time)\n", FILE_LOG);
    printf("  %s (human-readable summary)\n", FILE_SUMMARY);
    printf("Showing %d of the total events on console.\n", printed_events);

    free(samp.times);
    free(samp.sizes);

    if (ring.head) {
        Order *p = ring.head->next;
        while (p != ring.head) {
            Order *tmp = p;
            p = p->next;
            free(tmp);
        }
        free(ring.head);
    }

    return 0;
}
