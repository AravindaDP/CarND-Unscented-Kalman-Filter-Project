Feature: CalculateRMSE
  Calculate RMSE

  Scenario: When there are no estimations
    Given estimations length is 0
    And ground_truth length is 0
    When I calculate RMSE
    Then I should get "0 0 0 0" as output

  Scenario: When estimations size is different to ground_thruth
    Given estimations is
      """
      1 1 0.2 0.1
      """
    And ground_truth is
      """
      1.1 1.1 0.3 0.2
      2.1 2.1 0.4 0.3
      """
    When I calculate RMSE
    Then I should get "0 0 0 0" as output

  Scenario: Calculating RMSE
    Given estimations is
      """
      1 1 0.2 0.1
      2 2 0.3 0.2
      3 3 0.4 0.3
      """
    And ground_truth is
      """
      1.1 1.1 0.3 0.2
      2.1 2.1 0.4 0.3
      3.1 3.1 0.5 0.4
      """
    When I calculate RMSE
    Then I should get "0.1 0.1 0.1 0.1" as output