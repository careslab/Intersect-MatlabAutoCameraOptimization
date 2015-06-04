classdef PointOfView
    %PointOfView: This class contains the data to calculate camera view
    %points.
    % Abhilash Pandya
    % Create n 3D objects with importance values, create a viewing pyramid and
    %  optimize the view point.
    % usage:
    % [x y z] = sphere (5); where x y z are vectors of points.
    % you can choose any model and specify the vertices...sphere is just an
    % example.
    % Call structure.
    % pv = PointOfView()
    % pv = pv.AddObject(x,y,z, [3 3 3], 1, 10); % call the sphere before
    % example function calls
    % pv.CollisionPercentage();
    % pv.OptimizeLookAt(); optimizes the look at point.
    % pv.OptimizeLookFrom(); optimizes the look from point.
    % h = axes();
    % pv.DrawObjects(h)
    % pv.DrawViewPyramid(h)
    % Note: This is a class that will allow us to experiment with different
    % view optimization algorithms.
    
    properties
        % should make these private...
        num_objects;  % number of objects
        
        objects = struct ('x', 0, 'y', 0, 'z' , 0, 'num_rows',  0, 'scale' , 0 , 'pos', [0 0 0],'importance', 0);      % parameters of the objects
        
        pyramid ;           % contains the points of the view pyramid
  
        pyramid_dof;        % contains the points of the smaller depth of field
        
        pyramid_internal;   % this is the frustrum that is at the view to the 
                            % depth of field (the no man's zone).
        
        pyramid_lookat ;    % point where the view is pointing
        
        pyramid_pos ;       % point where the view starts
        
        field_of_view ;     % in degrees, the extent of the view
        
        flength ;           % focal length of the camers
        
        depth_of_field ;    % where the focus is valid
        
        hull;               % the hull for collisions.
    end % properties
    
    methods
        %% Constructor
        function h = PointOfView()
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %PointOfView() no input needed. output is the class.
            %Constructor function.
            % h is the handle of the class.
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %Put in the initial values of the view point
            h.num_objects = 0;  % total number of objects.
            
            h.pyramid_lookat = [ 3 3 3];
            
            h.pyramid_pos = [ 0 0 0];
            
            h.field_of_view = 35;
            
            h.flength = 5;
            
            h.depth_of_field = 2;
            
            objects = struct ('x', 0, 'y', 0, 'z' , 0, 'num_rows', 0,'scale' , 0 , 'pos', [0 0 0], 'importance', 0) ;      % parameters of the objects
            
            h.objects(1) = objects;
            
            % Create the two pyramids from thed default viewing pyramid.
            h.pyramid   = h.CreatePyramid(h.field_of_view,h.pyramid_pos,h.pyramid_lookat,h.flength,0);
            
            h.pyramid_dof  = h.CreatePyramid(h.field_of_view,h.pyramid_pos,h.pyramid_lookat,(h.flength - h.depth_of_field),0);
            
            h.pyramid_internal  = h.CreatePyramid(h.field_of_view,h.pyramid_pos,h.pyramid_lookat, h.depth_of_field,0);

            
            % compute the hull.
            h.hull = h.GetHull();
        end
        
        %% Object functions (objects/ view pyramid)
        function h = AddObject (h, x, y, z, pos, scale, importance)
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %AddObject ( x,y,z (the points), pos  ([ a b c]), scale (size), importance (1-10))
            %This function adds an object.
            % x y z are equal length vector of vertices.
            % pos: the position of this object
            % scale: size of the object
            % importance (scaled from 1 -10)
            % for now, the object type is the output of the sphere command.
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            h.num_objects = h.num_objects + 1;
            [a b] = size (x);  %this is because sphere returns a 2 D square matrix.
            num_rows = a * b;
            
            x = (x * scale + pos(1));
            y = (y * scale + pos(2));
            z = (z * scale + pos(3));
            
            h.objects(h.num_objects) = struct ('x',x, 'y', y, 'z', z,  'num_rows', num_rows, 'scale' , scale , 'pos', pos,'importance', importance);
        end
        
        
        
        function  [pyramid] = CreatePyramid(h, fov,P,Target,mag,portrait)
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % CreatePyramid( fov,position,lookat,mag,portrait (0 or 1))
            % This function creates the pyramid and stores the points
            % in the h.pyramid...given the conditions of the input.
            % from old Angott software written by A. Cao / A. Pandya
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            xc = [0 0 0 0;
                1 1 -1 1;
                -1 1 -1 -1];
            yc = [0 0 0 0;
                1 1 -1 -1;
                1 -1 1 -1];
            zc = [0 0 0 0;
                1 1 1 1;
                1 1 1 1];
            
            if portrait
                xc = xc*tand(fov/2)*3/4;
                yc = yc*tand(fov/2);
            else
                xc = xc*tand(fov/2);
                yc = yc*tand(fov/2)*3/4;
            end
            xc = xc*mag;
            yc = yc*mag;
            zc = zc*mag;
            
            % Rotation Matrix
            zdir = [0 0 1];
            Vz = Target-P;
            Vz = Vz./norm(Vz);
            Vx = cross(zdir,Vz);
            if isequal(Vx,[0 0 0])
                Vx = [1 0 0];
            end
            Vy = cross(Vz,Vx);
            R = [Vx'./norm(Vx) Vy'./norm(Vy) Vz'./norm(Vz)];
            
            xyz = [xc(:) yc(:) zc(:)];
            T = (R*xyz')';
            Tx = T(:,1);
            Ty = T(:,2);
            Tz = T(:,3);
            cx = reshape(Tx,size(xc,1),[]) + P(1);
            cy = reshape(Ty,size(yc,1),[]) + P(2);
            cz = reshape(Tz,size(zc,1),[]) + P(3);
            
            
            %just grab the unique rows.
            pyramid = struct ('x', cx, 'y', cy, 'z', cz);
            
        end
        
        function chull = GetHull(h)
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % GetHull()...given the current pyramid.
            % Get the depth of field... front and back face
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            nearRectangle = struct ('x',h.pyramid_dof.x, 'y', h.pyramid_dof.y, 'z', h.pyramid_dof.z);
            farRectangle = struct ('x',h.pyramid.x, 'y', h.pyramid.y, 'z', h.pyramid.z);
            
            %extract the coordinates of the convex hull at the end of the pyramid.
            chull = zeros(16, 3);
            k = 1;
            for i=2:3
                for j = 1:4
                    chull(k, :) = [ nearRectangle.x(i,j) nearRectangle.y(i,j) nearRectangle.z(i,j)];
                    k = k+1;
                end
                for j = 1:4
                    chull(k, :) = [ farRectangle.x(i,j) farRectangle.y(i,j) farRectangle.z(i,j)];
                    k = k + 1;
                end
            end
            chull = unique (chull, 'rows');
            
        end
        function ihull = GetiHull(h)
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % GetiHull()...given the current pyramid.
            % Get the depth of field... front and back face get the
            % interior hull
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            farRectangle = struct ('x',h.pyramid_dof.x, 'y', h.pyramid_dof.y, 'z', h.pyramid_dof.z);
            nearRectangle = struct ('x',h.pyramid_internal.x, 'y', h.pyramid_internal.y, 'z', h.pyramid_internal.z);
            
            %extract the coordinates of the convex hull at the end of the pyramid.
            ihull = zeros(16, 3);
            k = 1;
            for i=2:3
                for j = 1:4
                    ihull(k, :) = [ nearRectangle.x(i,j) nearRectangle.y(i,j) nearRectangle.z(i,j)];
                    k = k+1;
                end
                for j = 1:4
                    ihull(k, :) = [ farRectangle.x(i,j) farRectangle.y(i,j) farRectangle.z(i,j)];
                    k = k + 1;
                end
            end
            ihull = unique (ihull, 'rows');
            
        end
        function [ percentage, insidehull, outsidehull ] = CollisionPercentage (h)
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %This function will compute the % inview and return an error
            % on which optimization can be done.  It takes into
            % consideration importance and also how far the object is from
            % the centroid of the convex hul.
            % It also returns the percentage coverage, and the xyz points
            % inside the hull and those outside the hull.
            % ... it needs the function inhull function.
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            
            % compute the hull.
            chull = h.GetHull();
            ihull = h.GetiHull();
            %Check each point of the sphere for intersection.
            count = 0;
            %total points
            total_points = 0;
            %in/ outhull index
            in_count = 1;
            out_count = 1;
            
            insidehull = [];
            outsidehull =[];
            percentage = 0;
            for (k = 1: h.num_objects) %for each object.
                
                for (i = 1:h.objects(k).num_rows) %for each point
                    
                    % check if in convex hull?
                    if (inhull([h.objects(k).x(i),h.objects(k).y(i), h.objects(k).z(i)], chull))
                        insidehull(in_count, :) = [h.objects(k).x(i) h.objects(k).y(i) h.objects(k).z(i)];
                        in_count = in_count +1;
                        count = count + 1;
                        %check if there are points in the interior hull?
                    elseif (inhull([h.objects(k).x(i),h.objects(k).y(i), h.objects(k).z(i)], ihull))
                        outsidehull (out_count, :) = [h.objects(k).x(i) h.objects(k).y(i) h.objects(k).z(i)];
                        out_count = out_count+1;
                        
                    end
                    total_points = total_points +1;
                end %each point
                
                %Percentage.
                percentage = (count/total_points ) * 100;
                
            end  %for  each object
            
        end %function CollisionPercentage
        
        
        %% Optimization functions
        function h = OptimizeLookAt(h)
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % This is call function for the optimization.  It calls the
            % fminsearch with the objective function--> OptLookat which
            % intern calls PercentageObjectsInView
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            %copy over the parameter to be optimized
            pyramid_opt = h.pyramid_lookat;
            options = optimset('TolX',1, 'TolFun', 0.1, 'Display', 'iter');
            
            h.pyramid_lookat = fminsearch (@(pyramid_opt) ...
                h.ObjectiveFunctionLookat( pyramid_opt, h.pyramid_pos),pyramid_opt, options );
            
        end
        function h = OptimizeLookFrom(h)
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % This is the call function for optimizing the look from point
            % It calls the fminsearch with the objective function
            % -->OptLookat which in tern calls the PercentageObjectsInView
            %
            % Todo: restrict the pyramid position to be only in the view
            % direction to simulat zoom. This can be done by just scaling
            % the view vector.
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            %copy over the parameter to be optimized
            pyramid_opt = h.pyramid_pos;
            disp ('Begin************');
            h.pyramid_pos = fminsearch (@(pyramid_opt)...
                h.ObjectiveFunctionLookat( h.pyramid_lookat, pyramid_opt ),pyramid_opt );
            %update the pyramids
            % Create the two pyramids from the new viewing parameters.
            h.pyramid = h.CreatePyramid(h.field_of_view,h.pyramid_pos,h.pyramid_lookat,h.flength,0);
            h.pyramid_dof = h.CreatePyramid(h.field_of_view,h.pyramid_pos,h.pyramid_lookat,(h.flength - h.depth_of_field),0);
            
        end
        function h = OptimizeZoom(h)
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % This is the call function for optimizing the look from point
            % It calls the fminsearch with the objective function
            % -->OptLookat which in tern calls the PercentageObjectsInView
            %
            % Todo: restrict the pyramid position to be only in the view
            % direction to simulat zoom. This can be done by just scaling
            % the view vector.
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            %copy over the parameter to be optimized
            zoom = -0.5;
            % optimization parameters:
            options = optimset('TolX',1, 'TolFun', 0.1, 'Display', 'iter');
            disp ('Begin************');
            %zoom = ga (@(zoom)...
            zoom = fminsearch (@(zoom)... 
            h.ObjectiveFunctionZoom( zoom ),zoom, options );
            %update the pyramids
            % Create the two pyramids from the new viewing parameters.
            h.pyramid_pos = (h.pyramid_lookat - h.pyramid_pos)/ ...
                           norm((h.pyramid_lookat - h.pyramid_pos)) * zoom;
            h.pyramid = h.CreatePyramid(h.field_of_view,h.pyramid_pos,h.pyramid_lookat,h.flength,0);
            h.pyramid_dof = h.CreatePyramid(h.field_of_view,h.pyramid_pos,h.pyramid_lookat,(h.flength - h.depth_of_field),0);
            
        end
        function h = OptimizeZoomAndLookat(h)
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % This is the call function for optimizing the look from point
            % It calls the fminsearch with the objective function
            % -->OptLookat which in tern calls the PercentageObjectsInView
            %
            % Todo: restrict the pyramid position to be only in the view
            % direction to simulat zoom. This can be done by just scaling
            % the view vector.
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            %copy over the parameter to be optimized
            zoom_lookat(1) = -0.5;
            zoom_lookat(2:4) = h.pyramid_lookat;
            
            % optimization parameters:
            options = optimset('TolX',1, 'TolFun', 0.1, 'Display', 'iter');
            disp ('Begin************');
            %zoom = ga (@(zoom)...
            zoom_lookat = fminsearch (@(zoom_lookat)... 
            h.ObjectiveFunctionZoomAndLookat( zoom_lookat ),zoom_lookat, options ); % 
            %update the pyramids
            % Create the two pyramids from the new viewing parameters.
            h.pyramid_pos = (h.pyramid_lookat - h.pyramid_pos)/ ...
                           norm((h.pyramid_lookat - h.pyramid_pos)) * zoom_lookat(1);
            
            h.pyramid_lookat =   zoom_lookat(2:4);
            
            h.pyramid = h.CreatePyramid(h.field_of_view,h.pyramid_pos,h.pyramid_lookat,h.flength,0);
            h.pyramid_dof = h.CreatePyramid(h.field_of_view,h.pyramid_pos,h.pyramid_lookat,(h.flength - h.depth_of_field),0);
            
        end
        %% Corresponding Objective functions
        function [ error ] = ObjectiveFunctionLookat(h, pyramid_lookat, pyramid_pos )
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %This is the objective function called for optimization of the look at
            %point (angle) or the pyramid_pos.  Depending on which one the
            % algorithm uses to optimize.  Note that it calls the PercentageObjectsInView
            pyramid = h.CreatePyramid(h.field_of_view, pyramid_pos ,pyramid_lookat,h.flength,0);
            %get depth of field
            pyramid_dof = h.CreatePyramid(h.field_of_view,pyramid_pos,pyramid_lookat,(h.flength - h.depth_of_field),0);
            
            %Get the pyramid front and back face
            nR = struct ('x',pyramid_dof.x, 'y', pyramid_dof.y, 'z', pyramid_dof.z);
            fR = struct ('x',pyramid.x, 'y', pyramid.y, 'z', pyramid.z);
            
            %Compute the error for this view.
            error = h.PercentageObjectsInView(nR, fR);
            fprintf ('***Error %.5f\n pos %.5f %.5f %.5f\n look: %.5f %.5f %.5f \n', error, pyramid_pos, pyramid_lookat);
            
        end
        function [ error ] = ObjectiveFunctionZoom(h, zoom )
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %This is the objective function called for optimization of zoom at
            %pyramid_pos.  Depending on which one the
            % algorithm uses to optimize.  Note that it calls the PercentageObjectsInView
            view_vec = h.pyramid_lookat - h.pyramid_pos;
            pyramid_pos = (view_vec/norm(view_vec)) * zoom;
              
            pyramid = h.CreatePyramid(h.field_of_view, pyramid_pos ,h.pyramid_lookat,h.flength,0);
            %get depth of field
            pyramid_dof = h.CreatePyramid(h.field_of_view,pyramid_pos,h.pyramid_lookat,(h.flength - h.depth_of_field),0);
            pyramid_internal = h.CreatePyramid(h.field_of_view,pyramid_pos,h.pyramid_lookat,(h.depth_of_field),0);
            
            %Get the pyramid front and back face
            nR = struct ('x',pyramid_dof.x, 'y', pyramid_dof.y, 'z', pyramid_dof.z);
            fR = struct ('x',pyramid.x, 'y', pyramid.y, 'z', pyramid.z);
            iR = struct ('x',pyramid_internal.x, 'y', pyramid_internal.y, 'z', pyramid.z);
            %Compute the error for this view.
            error = h.PercentageObjectsInZoomView(nR, fR, iR);
            fprintf ('***Error %.5f\n pos %.5f %.5f %.5f\n look: %.5f %.5f %.5f \n', error, pyramid_pos, h.pyramid_lookat);
            
        end  
        function [ error ] = ObjectiveFunctionZoomAndLookat(h, zoom_lookat )
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %This is the objective function called for optimization of zoom at
            %pyramid_pos.  Depending on which one the
            % algorithm uses to optimize.  Note that it calls the PercentageObjectsInView
            view_vec = h.pyramid_lookat - h.pyramid_pos;
            pyramid_pos = (view_vec/norm(view_vec)) * zoom_lookat(1);
            h.pyramid_lookat = zoom_lookat (2:4);  
            pyramid = h.CreatePyramid(h.field_of_view, pyramid_pos ,h.pyramid_lookat,h.flength,0);
            %get depth of field
            pyramid_dof = h.CreatePyramid(h.field_of_view,pyramid_pos,h.pyramid_lookat,(h.flength - h.depth_of_field),0);
            pyramid_internal = h.CreatePyramid(h.field_of_view,pyramid_pos,h.pyramid_lookat,(h.depth_of_field),0);
            
            %Get the pyramid front and back face
            nR = struct ('x',pyramid_dof.x, 'y', pyramid_dof.y, 'z', pyramid_dof.z);
            fR = struct ('x',pyramid.x, 'y', pyramid.y, 'z', pyramid.z);
            iR = struct ('x',pyramid_internal.x, 'y', pyramid_internal.y, 'z', pyramid.z);
            %Compute the error for this view.
            error = h.PercentageObjectsInZoomView(nR, fR, iR);
            fprintf ('***Error %.5f\n pos %.5f %.5f %.5f\n look: %.5f %.5f %.5f \n', error, pyramid_pos, h.pyramid_lookat);
            
        end
        %% Functions used by objective function to compute percentage in view.
        function [ percentage ] = PercentageObjectsInView (h, nearRectangle, farRectangle)
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %This function will compute the % inview and return an error
            % on which optimization can be done.  It takes into
            % consideration importance and also how far the object is from
            % the centroid of the convex hul.
            % for now , I am assuming that the objects are all spheres
            % and have the same tesselation  (see the Sphere command).
            % it needs the function inhull.
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            %extract the coordinates of the convex hull at the end of the pyramid.
            chull = zeros(16, 3);
            k = 1;
            for i=2:3
                for j = 1:4
                    chull(k, :) = [ nearRectangle.x(i,j) nearRectangle.y(i,j) nearRectangle.z(i,j)];
                    k = k+1;
                end
                for j = 1:4
                    chull(k, :) = [ farRectangle.x(i,j) farRectangle.y(i,j) farRectangle.z(i,j)];
                    k = k + 1;
                end
            end
            chull = unique (chull, 'rows');
            
            % compute the centroid of the viewing fulcstrum
            centroid = sum (chull)/8;
            % compute the maximum distance of diagnol to the center.
            max_dist = pdist ([chull(8,:); chull(1,:)]);
            
            %Check each point of the sphere for intersection.
            count = 0;
            simple_count = 0;
            %total points
            total_points = 0;
            for (k = 1: h.num_objects) %for each object.
                
                for (i = 1:h.objects(k).num_rows) %for each point
                    
                    % check if in convex hull?
                    if (inhull([h.objects(k).x(i),h.objects(k).y(i), h.objects(k).z(i)], chull))
                        points = [h.objects(k).x(i) h.objects(k).y(i) h.objects(k).z(i); centroid];
                        dist = pdist (points, 'euclidean');
                        % add to previous....correction for how far the
                        % point is from the center, and how important
                        % it is.
                        count = count + (max_dist - dist) * h.objects(k).importance;
                        simple_count = simple_count +1;
                    end
                    total_points = total_points +1;
                end %each point
                %get all the importance values
                for (i=1:h.num_objects)
                    imp(i)= h.objects(i).importance;
                end
                
                %We are returning how much is not in view so we can minimize this.
                percentage = 100 - ((count/((max_dist + max (imp)) *(total_points )) * 100));
                %percentage = 100- ((simple_count/total_points)* 100);
            end  %for  each object
            
        end %function PercentageObjectsInView    
        function [ percentage ] = PercentageObjectsInZoomView (h, nearRectangle, farRectangle, internalRectangle)
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %This function will compute the % inview and return an error
            % on which optimization can be done.  It takes into
            % consideration importance and also how far the object is from
            % the centroid of the convex hul.
            % for now , I am assuming that the objects are all spheres
            % and have the same tesselation  (see the Sphere command).
            % it needs the function inhull.
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            %extract the coordinates of the convex hull at the end of the pyramid.
            chull = zeros(16, 3);
            k = 1;
            for i=2:3
                for j = 1:4
                    chull(k, :) = [ nearRectangle.x(i,j) nearRectangle.y(i,j) nearRectangle.z(i,j)];
                    k = k+1;
                end
                for j = 1:4
                    chull(k, :) = [ farRectangle.x(i,j) farRectangle.y(i,j) farRectangle.z(i,j)];
                    k = k + 1;
                end
            end
            
            chull = unique (chull, 'rows');
    
            %extract the points of the internal hull
            ihull = zeros(16, 3);
            k = 1;
            for i=2:3
                for j = 1:4
                    ihull(k, :) = [ nearRectangle.x(i,j) nearRectangle.y(i,j) nearRectangle.z(i,j)];
                    k = k+1;
                end
                for j = 1:4
                    ihull(k, :) = [ internalRectangle.x(i,j) internalRectangle.y(i,j) internalRectangle.z(i,j)];
                    k = k + 1;
                end
            end
            ihull = unique (ihull, 'rows');

            
            % compute the centroid of the viewing frustrum
            centroid = sum (chull)/8;
            % compute the maximum distance of diagnol to the center.
            max_dist = pdist ([chull(8,:); chull(1,:)]);
            
            
            % compute the maximum distance of diagnol to the center.
            imax_dist = pdist ([ihull(8,:); ihull(1,:)]);
   
            %Check each point of the sphere for intersection.
            count = 0;
            simple_count = 0;
            %total points
            total_points = 0;
            for (k = 1: h.num_objects) %for each object.
                
                for (i = 1:h.objects(k).num_rows) %for each point
                    
                    % check if in depth of field convex hull?
                    if (inhull([h.objects(k).x(i),h.objects(k).y(i), h.objects(k).z(i)], chull))
                        points = [h.objects(k).x(i) h.objects(k).y(i) h.objects(k).z(i); centroid];
                        dist = pdist (points, 'euclidean');
                        % add to previous....correction for how far the
                        % point is from the center, and how important
                        % it is.
                        count = count - (max_dist - dist) * h.objects(k).importance;
                        %count = count - 1;
                        simple_count = simple_count +1;
                        %check if the point is in the inner hull.
                    else if (inhull([h.objects(k).x(i),h.objects(k).y(i), h.objects(k).z(i)], ihull))
                            %if so, create a heavy penalty for this.
                             points = [h.objects(k).x(i) h.objects(k).y(i) h.objects(k).z(i); ihull(1,:)];
                             dist = pdist (points, 'euclidean');

                            count = count + ((imax_dist - dist) * h.objects(k).importance);
                            %count = count + 1;
                        end
                    end

                    total_points = total_points +1;
                end %each point
                %get all the importance values
                for (i=1:h.num_objects)
                    imp(i)= h.objects(i).importance;
                end
                
                %We are returning how much is not in view so we can minimize this.
                percentage = count /(imax_dist *(total_points )) * 100;
                %percentage = 100- ((simple_count/total_points)* 100);
                %percentage = count;
            end  %for  each object
            
        end %function PercentageObjectsInView
        %% Change values
        function h = Change_object_pos(h,object_num, pos)
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %Change_object_pos(object_num (which number?), pos (new pos [x yz])
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            if ((object_num > h.num_objects)||(object_num < 1 ))
                disp (' error on change object pos');
            else
                %substract the old pos, and add the new.
                h.objects(object_num).x = h.objects(object_num).x + (pos(1) - h.objects(object_num).pos(1));
                h.objects(object_num).y = h.objects(object_num).y + (pos(2) - h.objects(object_num).pos(2));
                h.objects(object_num).z = h.objects(object_num).z + (pos(3) - h.objects(object_num).pos(3));
                h.objects(object_num).pos = pos;
            end
            
        end
        
        function h = Change_pyramid_lookat (h, pyramid_lookat)
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Change_pyramid_lookat (pyramid_lookat ([ x y z])
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            h.pyramid_lookat = pyramid_lookat;
        end
        
        function h = Change_pyramid_pos(h,pyramid_pos)
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Change_pyramid_pos(pyramid_pos ([ x y z]) position
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            h.pyramid_pos= pyramid_pos;
        end
        
        function Change_field_of_view (h,field_of_view)
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Change_field_of_view (field_of_view (in degrees))
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            h.field_of_view= field_of_view;
        end
        
        function h = Change_flength (h,flength)
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Change_flength (flength)
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            h.flength = flength;
        end
        
        
        function h = Change_depth_of_field( h,depth_of_field)
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Change_depth_of_field(depth_of_field)
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            h.depth_of_field = depth_of_field;
        end
        
        
        %% Drawing routines
        function DrawViewPyramid(h, win)
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % DrawViewPyramid (win)..give it the window handle.
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % recreate the two pyramids from the new viewing parameters.
            h.pyramid = h.CreatePyramid(h.field_of_view,h.pyramid_pos,h.pyramid_lookat,h.flength,0);
            h.pyramid_dof = h.CreatePyramid(h.field_of_view,h.pyramid_pos,h.pyramid_lookat,(h.flength - h.depth_of_field),0);
            h.pyramid_internal  = h.CreatePyramid(h.field_of_view,h.pyramid_pos,h.pyramid_lookat, h.depth_of_field,0);

            axes (win);
            alpha ('clear');
            hP = fill3(h.pyramid.x,h.pyramid.y,h.pyramid.z,.01);
            hold on;
            hP = fill3(h.pyramid_dof.x,h.pyramid_dof.y,h.pyramid_dof.z,.01);
            hP = fill3(h.pyramid_internal.x,h.pyramid_internal.y,h.pyramid_internal.z,.01);
            alpha (0.1);
            
            %draw the view vector
            vectarrow(h.pyramid_pos, h.pyramid_lookat);
            hold off;
        end
        
        function DrawObjects(h, win)
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % DrawObjects(win)..give it the window handle.
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            if (h.num_objects == 0)
                disp ('no objects\n');
                return;
            end;
            
            axes (win);
            for (i = 1: h.num_objects)
                mesh(h.objects(i).x, h.objects(i).y, h.objects(i).z);
                hold on;
            end
            hold off;
        end
        
        function DrawInOutHullPoints(h, win, in, out)
            if (h.num_objects == 0 )
                disp ('no objects or collisions\n');
                return;
            end;
            
            axes (win);
            if (~isempty(in))
                plot3 (in(:,1), in(:,2), in(:,3), 'rs');
            end
            if (~isempty(out))
                plot3 (out(:,1), out(:,2), out(:,3), 'g*');
            end
        end
        
    end %methods
end %class

