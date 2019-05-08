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
    Then x_ with accuracy 0.00002 should be
      """
      -0.41614 0.90929 0 0 0
      """
    And P_ with accuracy 0.00000001 should be
      """
      0.36 0 0 0 0
      0 0.36 0 0 0
      0 0 9 0 0
      0 0 0 2.25 0
      0 0 0 0 0.0625
      """

  Scenario: Second Measurement is LASER
    Given a new UKF
    And a measurement
      """
      L	0.463227 0.607415	1477010443000000	0.4	0.5	5	0	0	0
      """
    And I called ProcessMeasurement
    And another measurement
      """
      L	0.968521 0.40545	1477010443100000	0.9	0.5	5	0	0	0
      """
    And Prediction would get called
    And UpdateLidar would get called
    When I call ProcessMeasurement
    Then Prediction should be called
    And It should have received delta_t as 0.1
    And UpdateLidar should be called
    And It should have received measurement in
      """
      L	0.968521 0.40545	1477010443100000	0.9	0.5	5	0	0	0
      """

  Scenario: Second Measurement is RADAR
    Given a new UKF
    And a measurement
      """
      R	0.898658 0.617674 1.7986	1477010443050000	0.6	0.5	5	0	0	0
      """
    And I called ProcessMeasurement
    And another measurement
      """
      R	0.910574 0.610537 1.46233	1477010443150000	1.1	0.5	5	0	0	0
      """
    And Prediction would get called
    And UpdateRadar would get called
    When I call ProcessMeasurement
    Then Prediction should be called
    And It should have received delta_t as 0.1
    And UpdateRadar should be called
    And It should have received measurement in
      """
      R	0.910574 0.610537 1.46233	1477010443150000	1.1	0.5	5	0	0	0
      """