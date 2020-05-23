function fig = LocateParts(fig,i)
% Import .ply
[fig.f,fig.v,fig.data] = plyread([i '.ply'],'tri');

% Count the number of vertex
fig.vertexCount = size(fig.v,1);

% Move center to specific location
fig.verts = fig.v;
try fig.verts = fig.v - repmat(fig.origin,fig.vertexCount,1); end

% Locate the piece in its position
fig.pieceTr = makehgtform('translate',fig.pos);
% try fig.pieceTr = fig.pieceTr * trotz(fig.angle - pi/2); end

fig.updatedPoints = [fig.pieceTr * [fig.verts,ones(fig.vertexCount,1)]']';

% Scale the colours to be 0-to-1 (they are originally 0-to-255
fig.vertexColours = [fig.data.vertex.red, fig.data.vertex.green, fig.data.vertex.blue] / 255;

% Plot the trisurf
fig.mesh_h = trisurf(fig.f,fig.updatedPoints(:,1),fig.updatedPoints(:,2), fig.updatedPoints(:,3) ...
    ,'FaceVertexCData',fig.vertexColours,'EdgeColor','interp','EdgeLighting','flat');

end