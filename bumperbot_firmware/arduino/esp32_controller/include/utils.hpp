#ifndef UTILS_HPP
#define UTILS_HPP

#include <cstdint>
#include <functional>
#include <type_traits>

namespace utils {

template <std::uint8_t Attempts, class F, class... Args, class R = std::invoke_result_t<F, Args...>>
    requires std::constructible_from<bool, R> and (Attempts > 0)
[[nodiscard]] constexpr R retry(F&& f, Args&&... args) noexcept {
    R result{};
    for (std::uint8_t i = 0; i < Attempts; ++i) {
        result = std::invoke(f, std::forward<Args>(args)...);
        if (result) {
            break;
        }
    }

    return result;
}

};  // namespace utils

#endif  // UTILS_HPP
