function [found] = findUndirectedEdge(edge, edges_id)

id1 = edge(1);
id2 = edge(2);

foundForward = find(edges_id(:,1)==id1 & edges_id(:,2)==id2);
if length(foundForward) == 1
  found = 1;
  return;
end

foundBack = find(edges_id(:,2)==id1 & edges_id(:,1)==id2);
if length(foundBack) == 1
  found = 1;
  return;
end

if length(foundForward) > 1 || length(foundBack) > 1 
  error('Multiple edge!')
end

found = 0;

end