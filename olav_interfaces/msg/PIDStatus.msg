# This message contains the internal state of a PID controller, including all
# its current parameters, terms, inputs and outputs.

# Controller setpoint.
float64 setpoint

# Controller feedback.
float64 feedback

# Controller output.
float64 output

# Output limiter.
bool use_output_limiter

# Controller maximum output.
float64 maximum_output

# Controller minimum output.
float64 minimum_output

# Whether the controller is using the output change limiter.
bool use_output_change_limiter

# Maximum output change in a controller tick.
float64 maximum_output_change

# Controller feedforward gain.
float64 feedforward_gain

# Controller feedforward term, scaled by the feedforward gain.
float64 feedforward_term

# Controller proportional gain.
float64 proportional_gain

# Controller proportional term, scaled by the proportional term.
float64 proportional_term

# Controller integral gain.
float64 integral_gain

# Controller integral term, scaled by the integral gain.
float64 integral_term

# Whether the controller is using the integral term limiter.
bool use_integral_term_limiter

# Maximum value of the integral term after scaling by the integral gain.
float64 maximum_integral_term

# Controller derivative gain.
float64 derivative_gain

# Controller derivative term, scaled by the derivative gain.
float64 derivative_term
