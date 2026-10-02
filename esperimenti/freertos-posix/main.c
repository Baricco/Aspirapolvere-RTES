/* Prova: un task periodico FreeRTOS (port POSIX) misura il ritardo del proprio risveglio. */
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <sched.h>
#include <sys/mman.h>
#include "FreeRTOS.h"
#include "task.h"

#define N 3000
static long lat_us[N];
static volatile int fatti;
static int64_t ns(void){ struct timespec t; clock_gettime(CLOCK_MONOTONIC,&t); return (int64_t)t.tv_sec*1000000000LL+t.tv_nsec; }

static void periodico(void *p){
    (void)p;
    TickType_t ultimo = xTaskGetTickCount();
    vTaskDelayUntil(&ultimo, 10);
    int64_t t0 = ns(); TickType_t k0 = ultimo;
    for (int i=0;i<N;i++){
        vTaskDelayUntil(&ultimo, pdMS_TO_TICKS(10));
        int64_t atteso = t0 + (int64_t)(ultimo-k0)*1000000LL;
        lat_us[i] = (long)((ns()-atteso)/1000);
        fatti=i+1;
    }
    long mn=lat_us[0],mx=lat_us[0]; double s=0; for(int i=0;i<N;i++){ if(lat_us[i]<mn)mn=lat_us[i]; if(lat_us[i]>mx)mx=lat_us[i]; s+=lat_us[i]; }
    /* deriva: ultimo campione, cioè quanto il "tempo FreeRTOS" si allontana da quello reale */
    printf("campioni %d: ritardo rispetto al tempo reale min %ld us, medio %.0f us, max %ld us, ultimo %ld us\n", N, mn, s/N, mx, lat_us[N-1]);
    long j=0; for(int i=1;i<N;i++){ long d=lat_us[i]-lat_us[i-1]; if(d<0)d=-d; if(d>j)j=d; }
    printf("jitter massimo tra risvegli consecutivi: %ld us\n", j);
    fflush(stdout); exit(0);
}
static void sfondo(void *p){ (void)p; volatile unsigned long x=0; for(;;){ x++; } }

int main(void){
    cpu_set_t c; CPU_ZERO(&c); CPU_SET(3,&c); sched_setaffinity(0,sizeof c,&c);
    struct sched_param sp={.sched_priority=80}; if (sched_setscheduler(0,SCHED_FIFO,&sp)) perror("sched");
    mlockall(MCL_CURRENT|MCL_FUTURE);
    xTaskCreate(periodico,"per",8192,NULL,3,NULL);
    xTaskCreate(sfondo,"bg",8192,NULL,1,NULL);
    vTaskStartScheduler();
    return 1;
}
