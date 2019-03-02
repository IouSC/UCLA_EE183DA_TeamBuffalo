function [q_near M I] = nearest(nodes, q_rand)
    dist_set = [];
    for j = 1:length(nodes)
        n = nodes(j);
        tmp = pdist([n.coord; q_rand]);
        dist_set(j) = tmp;
    end
    [M, I] = min(dist_set);
    q_near = nodes(I);
end