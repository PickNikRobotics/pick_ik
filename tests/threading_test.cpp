#include <catch2/catch_test_macros.hpp>

#include <fmt/core.h>
#include <rsl/queue.hpp>

#include <chrono>
#include <vector>
#include <thread>

// The function
auto make_thread_fn(rsl::Queue<int>& q, int val) {
    return [=, &q]() {
        fmt::print("Pushed value {} to queue\n", val);
        q.push(val);
    };
}

TEST_CASE("blah") {
    using namespace std::chrono_literals;
    fmt::print("Hello world\n");

    rsl::Queue<int> q;
    std::vector<std::thread> threads;

    size_t num_threads = 10;
    threads.reserve(num_threads);
    for (size_t i=0; i<num_threads; ++i) {
        auto val = static_cast<int>(i);
        threads.push_back(std::thread(make_thread_fn(q, val)));
    }

    int soln;
    while(q.size() < num_threads) {
        soln++;
    }

    for (auto& t : threads) {
        t.join();
    }

    while(!q.empty()) {
        fmt::print("Queue has value {}\n", q.pop().value());
    }

    REQUIRE(true);
}
