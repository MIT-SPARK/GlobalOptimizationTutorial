function sesync_data = convert2sesync(measurements)

sesync_data.R = {measurements.R};
sesync_data.t = {measurements.t};
sesync_data.kappa = {measurements.kappa};
sesync_data.tau = {measurements.tau};
edge_start = cell2mat( {measurements.i} ); 
edge_end = cell2mat( {measurements.j} );
sesync_data.edges = [edge_start', edge_end'];