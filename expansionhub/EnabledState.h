#pragma once

namespace eh {

class EnabledState {
   public:
    EnabledState() = default;
    ~EnabledState() noexcept;

    bool Initialize();

    bool IsEnabled() const;

    EnabledState(const EnabledState&) = delete;
    EnabledState(EnabledState&&) = delete;
    EnabledState& operator=(const EnabledState&) = delete;
    EnabledState& operator=(EnabledState&&) = delete;

   private:
    int fd = -1;
};
}  // namespace eh
