# message for communicating the status of the state estimator
Header header

# possible statuses
byte OK=0     # standard operation
byte WARN=1   # state estimator is not currently trustworthy
byte ERROR=2  # state estimator has encountered an unrecoverable error

byte status # status types enumerated above
