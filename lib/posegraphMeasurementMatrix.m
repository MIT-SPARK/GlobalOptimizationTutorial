function measurements = posegraphMeasurementMatrix(graph)
if(strcmpi(graph.format,'2d'))
    n_edges = size(graph.edges,1);
    measurements = arrayfun(@(x) struct('i',[],'j',[], ...
        'R', [], 't', [], 'tau', [], 'kappa', []), 1:n_edges);
    for i = 1:n_edges
        edge = graph.edges(i, :);
        R_ij = rot2D(edge(5));
        
        Omega = [edge(6) edge(7) edge(8); ...
            edge(7) edge(9) edge(10); ...
            edge(8) edge(10) edge(11)];
        Omega = [R_ij zeros(2,1); 0 0 1]' * Omega * [R_ij zeros(2,1); 0 0 1];
        if(min(eig(Omega)) < 0)
            warning('Error in full covariance construction')
            disp(min(eig(Omega)));
        end
        
        measurements(i).i = edge(1);
        measurements(i).j = edge(2);
        measurements(i).t = edge(3:4)';
        measurements(i).R = R_ij;
        measurements(i).Omega = Omega;
        measurements(i).tau = 2/trace(inv(Omega(1:2,1:2)));
        measurements(i).kappa = Omega(3,3);
        measurements(i).weight = 1;
    end
elseif(strcmpi(graph.format,'3d'))
    n_edges = size(graph.edges,2);
    measurements = arrayfun(@(x) struct('i',[],'j',[], ...
        'R', [], 't', [], 'tau', [], 'kappa', []), 1:n_edges);
    for i = 1:n_edges
        edge = graph.edges(i);
        measurements(i).i = edge.i;
        measurements(i).j = edge.j;
        measurements(i).t = edge.t;
        measurements(i).R = edge.R;
        measurements(i).Omega = edge.Info;
        measurements(i).tau = 3/trace(inv(edge.Info(1:3,1:3)));
        measurements(i).kappa = 3/(2*trace(inv(edge.Info(end-2:end,end-2:end))));
        measurements(i).weight = 1;
    end
else
    error('Unknown format...')
end
    

end

