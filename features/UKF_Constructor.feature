Feature: UKF Constructor
  As a developer
  I want UKF::UKF() to initialize parameters and matrices
  So that they have proper values

  Scenario: Creating a UKF
    Given a new UKF
    Then is_initialized_ should be false
    And n_x_ should be 5
    And n_aug_ should be 7
    And lambda_ should be -4
    And weights_ should be of size 15
    And Xsig_pred_ should be of size 5 by 15