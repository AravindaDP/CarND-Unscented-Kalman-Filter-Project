import sys

sys.path.append("./python")

from behave import *
from ukf.tools import Tools
from ukf.matrix import Matrix
from numpy.testing import assert_array_almost_equal
from helpers import *

@given(u'estimations length is 0')
def step_impl(context):
    context.estimations = []


@given(u'ground_truth length is 0')
def step_impl(context):
    context.ground_truth = []


@given(u'ground_truth is')
def step_impl(context):
    context.ground_truth = [parse_vector(line) for line in context.text.splitlines()]


@when(u'I calculate RMSE')
def step_impl(context):
    tools = Tools()
    context.result = tools.calculate_rmse(context.estimations,
                                          context.ground_truth)


@then(u'I should get "{rmse}" as output')
def step_impl(context, rmse):
    assert_array_almost_equal(context.result.value,
                              [[float(i)] for i in rmse.split()])


@given(u'estimations is')
def step_impl(context):
    context.estimations = [parse_vector(line) for line in context.text.splitlines()]