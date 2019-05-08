from ukf.matrix import Matrix
from math import sqrt

class Tools:
    def calculate_rmse(self, estimations, ground_truth):
        """A helper method to calculate RMSE."""
        rmse = Matrix([[]])
        rmse.zero(4, 1)

        # check the validity of the following inputs:
        # * the estimation vector size should not be zero
        # * the estimation vector size should equal ground truth vector size
        if len(estimations) != len(ground_truth) or not estimations:
            print("Invalid estimation or ground_truth data")
            return rmse

        #accumulate squared residuals
        for i in range(len(estimations)):
            residual = estimations[i] - ground_truth[i]

            #coefficient-wise multiplication
            residual = residual.cwise_product(residual)
            rmse += residual

        #calculate the mean
        rmse.value = [[i[0]/len(estimations)] for i in rmse.value]

        #calculate the squared root
        rmse.value = [[sqrt(i[0])] for i in rmse.value]

        #return the result
        return rmse