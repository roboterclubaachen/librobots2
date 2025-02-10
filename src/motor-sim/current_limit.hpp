#pragma once

namespace librobots2::motor_sim
{
class CurrentLimit
{
private:
	CurrentLimit() = default;
	~CurrentLimit() = default;

public:
	static void
	set(double ampere);

	static void
	unset();

	static double
	get();

private:
	static double currentLimit_;
};
}  // namespace librobots2::motor_sim