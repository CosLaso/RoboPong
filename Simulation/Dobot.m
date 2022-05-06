classdef Dobot < handle
    properties
        %> Robot model
        model;
        %>
        workspace = [-2 2 -2 2 -0.3 2];
        %> Flag to indicate if gripper is used
        useGripper = false;
    end
    methods %% Class for UR3 robot simulation

        function self = Dobot(useGripper)
            self.useGripper = useGripper;
            %> Define the boundaries of the workspace
            % robot =
            self.GetDobotRobot();
            % robot =
            self.PlotAndColourRobot();%robot,workspace);
        end

%% GetUR3Robot
% Given a name (optional), create and return a UR3 robot model

function GetDobotRobot(self)
%     if nargin < 1
        % Create a unique name (ms timestamp after 1ms pause)
        pause(0.001);
        name = ['Dobot_', datestr(now, 'yyyymmddTHHMMSSFFF')];
%     end
            L(1) = Link('d', 0.09, 'a', 0, 'alpha', pi/2, 'offset', 0, 'qlim', [-3*pi/4 3*pi/4]);
            L(2) = Link('d', 0, 'a', -0.152, 'alpha', 0,'offset', deg2rad(-32.5), 'qlim', [0,17*pi/36]);
            L(3) = Link('d', 0, 'a', -0.147, 'alpha', 0, 'offset', deg2rad(62.25), 'qlim', [-pi/18,19*pi/36]);
            L(4) = Link('d', 0, 'a', 0.02, 'alpha', 0, 'offset', deg2rad(-30), 'qlim', [-pi/2,pi/2]);
            self.model = SerialLink(L, 'name', name);

            % Rotate robot to the correct orientation
            self.model.base = transl(0.55,0,1.145) * trotx(0,'deg') * troty(0,'deg');
end

        %% PlotAndColourRobot

        % Given a robot index, add the glyphs (vertices and faces) and
        % colour them in if data is available 

        function PlotAndColourRobot(self)%robot,workspace)
            for linkIndex = 0:self.model.n
                if self.useGripper && linkIndex == self.model.n
                    [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['DobotMagicianLink', num2str(linkIndex), 'Gripper.ply'], 'tri'); %#ok<AGROW>
                else
                    [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['DobotMagicianLink', num2str(linkIndex), '.ply'], 'tri'); %#ok<AGROW>
                end
                self.model.faces{linkIndex + 1} = faceData;
                self.model.points{linkIndex + 1} = vertexData;
            end

            % Display robot
            self.model.plot3d(zeros(1, self.model.n), 'noarrow', 'workspace', self.workspace);
            if isempty(findobj(get(gca, 'Children'), 'Type', 'Light'))
                camlight
            end  
            self.model.delay = 0;

            % Try to correctly colour the arm (if colours are in ply file data)
            for linkIndex = 0:self.model.n
                handles = findobj('Tag', self.model.name);
                h = get(handles, 'UserData');
                try 
                    h.link(linkIndex + 1).Children.FaceVertexCData = [plyData{linkIndex + 1}.vertex.red ...
                                                                  , plyData{linkIndex + 1}.vertex.green ...
                                                                  , plyData{linkIndex + 1}.vertex.blue]/255;
                    h.link(linkIndex + 1).Children.FaceColor = 'interp';
                catch ME_1
                    disp(ME_1);
                    continue;
                end
            end
        end
        
    end
end