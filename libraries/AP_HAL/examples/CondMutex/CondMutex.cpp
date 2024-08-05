/*
  test of HAL_BinarySemaphore
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Semaphores.h>
#include <AP_HAL/CondMutex.h>

#include <stdio.h>

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

//#define USE_COND_MUTEX

static uint8_t get_random(void)
{
    static uint32_t m_w = 76542;
    m_w = 18000 * (m_w & 65535) + (m_w >> 16);
    return m_w & 0xFF;
}

class ProducerConsumerTest {
public:
    HAL_BinarySemaphore sem1;
    HAL_CondMutex cmtx;

    void setup(void);
    void thread1(void);
    void thread2(void);
    void update(bool ok);
    void update_sent();
    bool check() { return bsize>0; }

    uint32_t ops, timeouts, sent;
    uint32_t bsize;
    uint32_t last_print_us;
    HAL_Semaphore mtx;
};

void ProducerConsumerTest::setup(void)
{
    hal.scheduler->thread_create(
        FUNCTOR_BIND_MEMBER(&ProducerConsumerTest::thread1, void), "thd1", 2048, AP_HAL::Scheduler::PRIORITY_IO, 0);
    hal.scheduler->thread_create(
        FUNCTOR_BIND_MEMBER(&ProducerConsumerTest::thread2, void), "thd2", 2048, AP_HAL::Scheduler::PRIORITY_IO, 0);
    ::printf("Setup threads\n");
}

void ProducerConsumerTest::thread2(void)
{
    while (true) {
#ifdef USE_COND_MUTEX
        cmtx.lock_and_wait(FUNCTOR_BIND_MEMBER(&ProducerConsumerTest::check, bool));
        bool ok = true;
        while (bsize>0) {
            update(ok);
        }
        cmtx.unlock();
#else
        bool ok = sem1.wait(50000);
        update(ok);
#endif
        hal.scheduler->delay_microseconds(get_random());
    }
}

void ProducerConsumerTest::thread1(void)
{
    while (true) {
#ifdef USE_COND_MUTEX
        cmtx.lock_and_signal();
        update_sent();
        cmtx.unlock();
#else
        sem1.signal();
        update_sent();
#endif
        hal.scheduler->delay_microseconds(get_random());
    }
}

void ProducerConsumerTest::update(bool ok)
{
    WITH_SEMAPHORE(mtx);
    if (ok) {
        ops++;
        bsize--;
    } else {
        timeouts++;
    }
    uint32_t now_us = AP_HAL::micros();
    float dt = (now_us - last_print_us)*1.0e-6;
    if (dt >= 1.0) {
        last_print_us = now_us;
        ::printf("tick %u %.3f ops/s %.3f timeouts/s missing %d\n",
                 unsigned(AP_HAL::millis()),
                 ops/dt,
                 timeouts/dt,
                 int32_t(sent) - int32_t(ops));
        ops = 0;
        sent = 0;
        timeouts = 0;
    }
}

void ProducerConsumerTest::update_sent()
{
    WITH_SEMAPHORE(mtx);
    sent++;
    bsize++;
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
