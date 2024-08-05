/*
  test of HAL_BinarySemaphore
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Semaphores.h>
#include <AP_HAL/CondMutex.h>
#include <AP_Math/AP_Math.h>

#include <stdio.h>

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#define USE_COND_MUTEX

class ProducerConsumerTest {
public:
    HAL_BinarySemaphore sem1;
    HAL_CondMutex cmtx;

    void setup(void);
    void producer(void);
    void consumer(void);
    void producer_tick(void);
    void consumer_tick(void);
    bool update();
    void update_sent();
    bool check() { return bsize>0; }

    uint32_t ops, timeouts, sent, recv;
    uint32_t bsize;
    uint32_t last_print_us;
    uint32_t last_sent_us;
    uint32_t max_delayed_us;
    HAL_Semaphore mtx;
    uint32_t delay_count;
};

void ProducerConsumerTest::setup(void)
{
    hal.scheduler->thread_create(
        FUNCTOR_BIND_MEMBER(&ProducerConsumerTest::producer, void), "producer", 2048, AP_HAL::Scheduler::PRIORITY_IO, 0);
    hal.scheduler->thread_create(
        FUNCTOR_BIND_MEMBER(&ProducerConsumerTest::consumer, void), "consumer", 2048, AP_HAL::Scheduler::PRIORITY_IO, 1);
    ::printf("Setup threads\n");
}

void ProducerConsumerTest::consumer(void)
{
    while (true) {
        consumer_tick();
    }
}

void ProducerConsumerTest::consumer_tick(void)
{
#ifdef USE_COND_MUTEX
    cmtx.lock_and_wait(FUNCTOR_BIND_MEMBER(&ProducerConsumerTest::check, bool));
    update();
    cmtx.unlock();
#else
    if (!check()) {
        sem1.wait_blocking();
    }
    if (!update()) {
        return;   // we thought we had a sample, but concurrency means we actually do not
    }
#endif
    hal.scheduler->delay_microseconds(100); // simluate processing delay
}

void ProducerConsumerTest::producer(void)
{
    while (true) {
        // simulate fifo burst
        producer_tick();
        producer_tick();
        producer_tick();
        hal.scheduler->delay_microseconds(750);
    }
}

void ProducerConsumerTest::producer_tick(void)
{
#ifdef USE_COND_MUTEX
    cmtx.lock_and_signal();
    update_sent();
    cmtx.unlock();
#else
    update_sent();
    sem1.signal();
#endif
}

bool ProducerConsumerTest::update()
{
    WITH_SEMAPHORE(mtx);
    // safety check
    if (!check()) {
        return false;
    }
    ops++;
    recv++;
    bsize--;
    uint32_t now_us = AP_HAL::micros();
    max_delayed_us = MAX(max_delayed_us, now_us - last_sent_us);
    float dt = (now_us - last_print_us)*1.0e-6;
    if (dt >= 1.0) {
        last_print_us = now_us;
        ::printf("tick %u %.3f ops/s, dt %uus missing %d max_delay %uus queue length %u\n",
                 unsigned(AP_HAL::millis()),
                 ops/dt,
                 uint32_t(dt * 1.0e6 / ops),
                 int32_t(sent) - int32_t(recv),
                 max_delayed_us,
                 bsize);
        ops = 0;
        max_delayed_us = 0;
    }
    return true;
}

void ProducerConsumerTest::update_sent()
{
    WITH_SEMAPHORE(mtx);
    sent++;
    bsize++;
    last_sent_us = AP_HAL::micros();
}

static ProducerConsumerTest *ct;

void setup(void)
{
    ct = new ProducerConsumerTest;
    ct->setup();
}

void loop(void)
{
    hal.scheduler->delay(1000);
}

AP_HAL_MAIN();
