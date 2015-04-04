Feature: Checking next state transtitions based on triggers

    Scenario Outline: Move to next state
        Given the robot is in <current> state
        When <trigger>
        Then transition to <next> state


    Examples: State 
        |current            |trigger          |next               |
        |off                |wake_up          |init               |
        |init               |booted           |exploration        |
        |exploration        |victim_found     |identification     |
        |exploration        |map_covered      |end                |
        |identification     |abort_victim     |victim_deletion    |
        |identification     |valid_victim     |closeup            |
        |closeup            |timeout          |fusion_validation  |
        |closeup            |verified         |operator_validation|
        |victim_deletion    |victim_deleted   |exploration        |
        |fusion_validation  |victim_classified|exploration        |
        |operator_validation|responded        |fusion_validation  |
        

