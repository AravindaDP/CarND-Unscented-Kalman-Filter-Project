Feature: Process Measurement
  As a developer
  I want UKF::ProcessMeasurement function
  So that I can process sensor measurements

  Scenario: First Measurement is LASER
    Given a new UKF
    And a measurement
      """
      L	1	1	0	1	1	0	0	0	0
      """
    When I call ProcessMeasurement
    Then x_ with accuracy 0.00000001 should be
      """
      1 1 0 0 0
      """
    And P_ with accuracy 0.00000001 should be
      """
      0.0225 0 0 0 0
      0 0.0225 0 0 0
      0 0 9 0 0
      0 0 0 2.25 0
      0 0 0 0 0.0625
      """

  Scenario: First Measurement is RADAR
    Given a new UKF
    And a measurement
      """
      R	1	2	0.5	0	-0.41614	0.90929	0.5	0.5	0	0
      """
    When I call ProcessMeasurement
    Then x_ with accuracy 0.00001 should be
      """
      -0.41614 0.90929 0 0 0
      """
    And P_ with accuracy 0.00000001 should be
      """
      0.09 0 0 0 0
      0 0.09 0 0 0
      0 0 9 0 0
      0 0 0 2.25 0
      0 0 0 0 0.0625
      """