import unittest
from ukf.tools import Tools
from ukf.matrix import Matrix
from numpy.testing import assert_array_almost_equal

class TestTools(unittest.TestCase):
    def setUp(self):
        self._tools = Tools()

    def test_CalculateRMSE_Returns0_IfEstimationsSizeIs0(self):
        estimations = []
        ground_truth = []
        expected_rmse = Matrix([[]])
        expected_rmse.zero(4, 1)

        rmse = self._tools.calculate_rmse(estimations, ground_truth)

        self.assertEqual(rmse.value, expected_rmse.value,
                         "RMSE should be 0 when estimation size is 0.")

    def test_CalculateRMSE_Returns0_IfEstimationsSizeDiffer(self):
        estimations = [Matrix([[1], [1], [0.2], [0.1]])]
        ground_truth = [Matrix([[1.1], [1.1], [0.3], [0.2]]),
                        Matrix([[2.1], [2.1], [0.4], [0.3]])]
        expected_rmse = Matrix([[]])
        expected_rmse.zero(4, 1)

        rmse = self._tools.calculate_rmse(estimations, ground_truth)

        self.assertEqual(rmse.value, expected_rmse.value,
                         "RMSE should be 0 when estimation size differs.")

    def test_CalculateRMSE_ReturnsCorrectRMSE_ForTestEstimations(self):
        for estimations, ground_truth, expected_rmse in [([Matrix([[1], [1], [0.2], [0.1]]),
                                                           Matrix([[2], [2], [0.3], [0.2]]),
                                                           Matrix([[3], [3], [0.4], [0.3]])],
                                                          [Matrix([[1.1], [1.1], [0.3], [0.2]]),
					                                       Matrix([[2.1], [2.1], [0.4], [0.3]]),
									                       Matrix([[3.1], [3.1], [0.5], [0.4]])],
                                                           Matrix([[0.1], [0.1], [0.1], [0.1]]))]:
            with self.subTest():
                rmse = self._tools.calculate_rmse(estimations, ground_truth)

                assert_array_almost_equal(rmse.value, expected_rmse.value,
                                          err_msg="RMSE should be correctly calculated.")