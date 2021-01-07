uint8 NONE = 0      # This mode turns the triggering off and prevents the sonar from pinging
                    #   and collecting data. This is the default mode.
uint8 MANUAL = 1    # This mode turns the triggering ON. The trigger is armed via the internal ping rate timer 
                    #   and gated by the external SYNC input. When the ping rate timer has armed the sonar and 
                    #   SYNC input is HIGH the sonar will transmit and collect one ping of data.
uint8 AUTO = 2      # This mode turns the triggering ON. The trigger is armed and tripped via the internal ping rate timer.
uint8 ONCE = 3       # This mode turns the trigger ON for one ping. The sonar will ping and collect data for one ping only
                    #   when the internal ping rate timer is expired.

uint8 TriggerMode
