# This message contains the internal state of a PID controller, including all
# its current parameters, terms, inputs and outputs.

# Controller setpoint.
float64 setpoint

# Controller feedback.
float64 feedback

# Controller output.
float64 output

# Controller feedforward term, scaled by the feedforward gain.
float64 feedforward_term

# Controller proportional term, scaled by the proportional term.
float64 proportional_term

# Controller integral term, scaled by the integral gain.
float64 integral_term

# Controller derivative term, scaled by the derivative gain.
float64 derivative_term
