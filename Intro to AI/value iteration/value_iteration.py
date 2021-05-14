import numpy
import vi_util
import copy


class State:
    def __init__(self, x, y):
        self.x = x
        self.y = y


stateList = [State(1, 1), State(2, 1), State(3, 1), State(4, 1),
             State(1, 2), State(3, 2), State(4, 2),
             State(1, 3), State(2, 3), State(3, 3), State(4, 3)]

A = ['u', 'r', 'd', 'l']

rewardArray_1 = numpy.array([-0.04, -0.04, -0.04, -0.04, -0.04, -0.04,
                             -1, -0.04, -0.04, -0.04, 1])
rewardArray_2 = numpy.array([-0.25, -0.25, -0.25, -0.04, -0.25, -0.25,
                             -1, -0.25, -0.25, -0.25, 1])
rewardArray_3 = numpy.array([-0.01, -0.01, -0.01, -0.01, -0.01, -0.01,
                             -1, -0.01, -0.01, -0.01, 1])
transition = vi_util.P


# S- outcomes, s'
# U- utilities
# P- transition model P(s'| s,a)
def getExpectedUtility(action, state, S, U, P):
    index = vi_util.getIndexOfState(S, state.x, state.y)
    E = 0

    for statePrimeIndex, statePrime in enumerate(S):
        if P[action, index, statePrimeIndex] > 0:
            E += P[action, index, statePrimeIndex] * U[statePrimeIndex]
    return E


def valueIteration(S, A, P, R_states, discount):
    U = numpy.full((len(S), 1), 0.0)

    U[6] = -1
    U[10] = 1
    U_prime = copy.deepcopy(U)

    threshold = 0.01 * (1 - discount) / discount

    while True:
        U = copy.deepcopy(U_prime)
        maxChange = 0
        for stateIndex, state in enumerate(S):
            r = R_states[stateIndex]
            if (stateIndex is not 6 and stateIndex is not 10):
                highestAU = -100
                for actionIndex, action in enumerate(A):
                    temp = getExpectedUtility(actionIndex, state, S, U, P)

                    if temp > highestAU:
                        highestAU = temp

                U_prime[stateIndex] = r + discount * highestAU

                if abs(U_prime[stateIndex] - U[stateIndex]) > maxChange:
                    maxChange = abs(U_prime[stateIndex] - U[stateIndex])
        if (maxChange < threshold or maxChange == 0):
            break

    return U


if __name__ == '__main__':


    print('Discount = 1.0 Reward = -0.04')
    U_1 = valueIteration(stateList, A, transition, rewardArray_1, 1)
    policy_1 = vi_util.getPolicyForGrid(stateList, U_1, A, transition, [6, 10])
    print(U_1)
    print("Policy:", policy_1)
    print(vi_util.printPolicyForGrid(policy_1, 4, 3, [5]))

    print('Discount = 1.0 Reward = -0.25')
    U_2 = valueIteration(stateList, A, transition, rewardArray_2, 1)
    policy_2 = vi_util.getPolicyForGrid(stateList, U_2, A, transition, [6, 10])
    print(U_2)
    print("Policy:", policy_2)
    print(vi_util.printPolicyForGrid(policy_2, 4, 3, [5]))

    print('Discount = 1.0 Reward = -0.01')
    U_3 = valueIteration(stateList, A, transition, rewardArray_3, 1)
    policy_3 = vi_util.getPolicyForGrid(stateList, U_3, A, transition, [6, 10])
    print(U_3)
    print("Policy:", policy_3)
    print(vi_util.printPolicyForGrid(policy_3, 4, 3, [5]))

    print('Discount = 0.5 Reward = -0.04')
    U_4 = valueIteration(stateList, A, transition, rewardArray_1, 0.5)
    policy_4 = vi_util.getPolicyForGrid(stateList, U_4, A, transition, [6, 10])
    print(U_4)
    print("Policy:", policy_4)
    print(vi_util.printPolicyForGrid(policy_4, 4, 3, [5]))

    print('Discount = 0.5 Reward = -0.25')
    U_5 = valueIteration(stateList, A, transition, rewardArray_2, 0.5)
    policy_5 = vi_util.getPolicyForGrid(stateList, U_5, A, transition, [6, 10])
    print(U_5)
    print("Policy:", policy_5)
    print(vi_util.printPolicyForGrid(policy_5, 4, 3, [5]))

    print('Discount = 0.5 Reward = -0.01')
    U_6 = valueIteration(stateList, A, transition, rewardArray_3, 0.5)
    policy_6 = vi_util.getPolicyForGrid(stateList, U_6, A, transition, [6, 10])
    print(U_6)
    print("Policy:", policy_6)
    print(vi_util.printPolicyForGrid(policy_6, 4, 3, [5]))
