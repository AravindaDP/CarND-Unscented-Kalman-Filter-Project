Feature: Predict Radar Measurement
  As a developer
  I want UKF::PredictRadarMeasurement function
  So that I can calculate mean predicted measurement and measurement covariance matrix
  from predicted sigma points

  Scenario: Predicting Radar Measurement
    Given a new UKF
    And std_radr_ is 0.3
    And std_radphi_ is 0.0175
    And std_radrd_ is 0.1
    And Xsig_pred_ is
      """
      5.9374  6.0640   5.925  5.9436  5.9266  5.9374  5.9389  5.9374  5.8106  5.9457  5.9310  5.9465  5.9374  5.9359  5.93744
      1.48  1.4436   1.660  1.4934  1.5036    1.48  1.4868    1.48  1.5271  1.3104  1.4787  1.4674    1.48  1.4851    1.486
      2.204  2.2841  2.2455  2.2958   2.204   2.204  2.2395   2.204  2.1256  2.1642  2.1139   2.204   2.204  2.1702   2.2049
      0.5367 0.47338 0.67809 0.55455 0.64364 0.54337  0.5367 0.53851 0.60017 0.39546 0.51900 0.42991 0.530188  0.5367 0.535048
      0.352 0.29997 0.46212 0.37633  0.4841 0.41872   0.352 0.38744 0.40562 0.24347 0.32926  0.2214 0.28687   0.352 0.318159
      """
    When I call PredictRadarMeasurement
    Then vector z_pred with accuracy 0.00005 should be
      """
      6.12155, 0.245993, 2.10313
      """
    And matrix S with accuracy 0.0000005 should be
      """
      0.0946171 -0.000139448   0.00407016
      -0.000139448  0.000617548 -0.000770652
      0.00407016 -0.000770652    0.0180917
      """