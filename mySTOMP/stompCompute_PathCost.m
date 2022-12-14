function Cost = stompCompute_PathCost(theta, obsts, hole, R, Env, nSamples)
%Compute the overall cost of a path


Costi = stompCompute_Cost(theta, obsts, hole, Env);
theta = theta(:, 2:nSamples-1);
Cost = sum(Costi) + 1/2 * sum(sum(theta * R * theta'));

end