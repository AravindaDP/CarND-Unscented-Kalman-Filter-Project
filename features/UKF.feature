Feature: Accuracy Rubric
  As a student
  I want a tuned UKF
  So that I can have RMSE less than or equal to the values [.09, .10, .40, .30]
  against Dataset 1 

  Scenario: Dataset 1
    Given a new UKF
    And Dataset 1 is used
    When I run UnscentedKF
    Then final rmse should be less than
      """
      0.09 0.10 0.40 0.30
      """