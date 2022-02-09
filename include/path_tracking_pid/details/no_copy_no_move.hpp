#pragma once

namespace path_tracking_pid::details
{

// Helper class to mark a class as non-copyable and non-moveable. Use private inheritance.
class NoCopyNoMove
{
public:
  NoCopyNoMove(const NoCopyNoMove&) = delete;
  NoCopyNoMove(NoCopyNoMove&&) = delete;
  NoCopyNoMove& operator=(const NoCopyNoMove&) = delete;
  NoCopyNoMove& operator=(NoCopyNoMove&&) = delete;

protected:
  NoCopyNoMove() = default;
  ~NoCopyNoMove() = default;
};

}  // namespace path_tracking_pid::details
