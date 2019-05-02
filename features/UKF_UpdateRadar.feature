Feature: Update Radar
  As a developer
  I want UKF::UpdateRadar function
  So that I can update the belief about object's position, state vector (x_)
  and covariance (P_) using radar data

  Scenario: Update Radar
    Given a new UKF
    And a measurement
      """
      R	5.9214	0.2187	2.0062	0	6.0	0.2	2.5	0.5	0	0
      """
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
    And x_ is
      """
      5.93637 1.49035 2.20528 0.536853 0.353577
      """
    And P_ is
      """
      0.00543425   -0.0024053   0.00341576  -0.00348196  -0.00299378
      -0.0024053     0.010845    0.0014923   0.00980182   0.00791091
      0.00341576    0.0014923   0.00580129  0.000778632  0.000792973
      -0.00348196   0.00980182  0.000778632    0.0119238    0.0112491
      -0.00299378   0.00791091  0.000792973    0.0112491    0.0126972
      """
    And PredictRadarMeasurement would get called
    And It would return Zsig as
      """
      6.1190  6.2334  6.1531  6.1283  6.1143  6.1190  6.1221  6.1190  6.0079  6.0883  6.1125  6.1248  6.1190  6.1188  6.12057
      0.24428  0.2337 0.27316 0.24616 0.24846 0.24428 0.24530 0.24428 0.25700 0.21692 0.24433 0.24193 0.24428 0.24515 0.245239
      2.1104  2.2188  2.0639   2.187  2.0341  2.1061  2.1450  2.1092  2.0016   2.129  2.0346  2.1651  2.1145  2.0786  2.11295
      """
    And It would return z_pred as
      """
      6.12155 0.245993 2.10313
      """
    And It would return S as
      """
      0.0946171 -0.000139448   0.00407016
      -0.000139448  0.000617548 -0.000770652
      0.00407016 -0.000770652    0.0180917
      """
    When I call UpdateRadar
    Then x_ with accuracy 0.00005 should be
      """
      5.92276 1.41823 2.15593 0.489274 0.321338
      """
    And P_ with accuracy 0.000001 should be
      """
      0.00361579 -0.000357881   0.00208316 -0.000937196  -0.00071727
      -0.000357881   0.00539867   0.00156846   0.00455342   0.00358885
      0.00208316   0.00156846   0.00410651   0.00160333   0.00171811
      -0.000937196   0.00455342   0.00160333   0.00652634   0.00669436
      -0.00071719   0.00358884   0.00171811   0.00669426   0.00881797
      """