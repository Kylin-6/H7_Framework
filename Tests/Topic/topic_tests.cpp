#include "topic.h"

#include <cstdio>
#include <cstdlib>

uint32_t test_primask;
static uint64_t test_now_us;

extern "C" uint64_t SYS_Timestamp_Get_Microsecond(void)
{
    return test_now_us;
}

static unsigned checks;
#define CHECK(condition) do { ++checks; if (!(condition)) { \
    std::fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #condition); \
    std::exit(1); } } while (0)

int main()
{
    Topic<uint32_t> topic;
    uint32_t value = 7U;
    CHECK(!topic.ReadFresh(value, 100U));
    CHECK(value == 7U);

    test_now_us = 1000U;
    topic.Publish(42U);
    test_now_us = 1100U;
    CHECK(topic.ReadFresh(value, 100U));
    CHECK(value == 42U);

    test_now_us = 1101U;
    value = 9U;
    CHECK(!topic.ReadFresh(value, 100U));
    CHECK(value == 9U);

    test_now_us = 1200U;
    topic.Publish(84U);
    CHECK(topic.ReadFresh(value, 100U));
    CHECK(value == 84U);

    test_now_us = 1199U;
    CHECK(!topic.ReadFresh(value, 100U));

    std::printf("PASS topic freshness: %u checks\n", checks);
    return 0;
}
