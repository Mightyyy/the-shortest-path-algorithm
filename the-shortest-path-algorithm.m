classdef The_shortest_path_algorithm < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        TheShortestPathAlgorithmUIFigure  matlab.ui.Figure
        InputMenu                       matlab.ui.container.Menu
        InputfromfileMenu               matlab.ui.container.Menu
        InputmanuallyMenu               matlab.ui.container.Menu
        GeneraterandomInputMenu         matlab.ui.container.Menu
        LoadsampleInputMenu             matlab.ui.container.Menu
        SaveMenu                        matlab.ui.container.Menu
        SaveDijkstraAlgorithmresulttofileMenu  matlab.ui.container.Menu
        SaveBellmanFordAlgorithmresulttofileMenu  matlab.ui.container.Menu
        SaveAdjacentMatrixtofileMenu    matlab.ui.container.Menu
        RunDijkstraalgorithmButton      matlab.ui.control.Button
        NotesTextAreaLabel              matlab.ui.control.Label
        NotesTextArea                   matlab.ui.control.TextArea
        Image                           matlab.ui.control.Image
        RunBellmanFordalgorithmButton   matlab.ui.control.Button
        ShowNetworkTopologyButton       matlab.ui.control.Button
        NumberofnodesEditFieldLabel     matlab.ui.control.Label
        NumberofnodesEditField          matlab.ui.control.NumericEditField
        RunDialalgorithmButton          matlab.ui.control.Button
        RunSPFAButton                   matlab.ui.control.Button
        ShowAdjacentMatrixButton        matlab.ui.control.Button
        SourceDestinationnodeEditFieldLabel  matlab.ui.control.Label
        SourceDestinationnodeEditField  matlab.ui.control.NumericEditField
        SamplegraphLabel                matlab.ui.control.Label
    end

    
    methods (Access = private)
        
    end
    

    % Callbacks that handle component events
    methods (Access = private)

        % Button pushed function: ShowNetworkTopologyButton
        function ShowNetworkTopologyButtonPushed(app, event)
            global input_graph;
            global graph_dimension;
            %         end
            %             global value1;
            % %             global G; %announce as global variables to share the value between functions
            %             s=[value1(1)]; %node1=s=input(3n+1)
            %             t=[value1(2)]; %node2=t=input(3n+2)
            s=[];
            t=[];
            weights=[];
            %             weights=[value1(3)]; %input(3n)=weight
            %             for i=4:3:length(value1) %form adjacent matrix
            %                 s=[s,value1(i)];%node s
            %                 t=[t,value1(i+1)];%node t
            %                 weights = [weights,value1(i+2)];%weight between s and t
            %             end
            for i=1:graph_dimension
                for j=1:i-1
                    if (input_graph(i,j)==inf)
                        continue
                    end
                    s=[s,i];%node s
                    t=[t,j];%node t
                    weights = [weights,input_graph(i,j)];%weight between s and t
                end
            end
            G = graph(s,t,weights) %form graph object
            plot(G,'EdgeLabel',G.Edges.Weight) %figure
        end

        % Button pushed function: RunDijkstraalgorithmButton
        function RunDijkstraalgorithmButtonPushed(app, event)
            %import graph information
            global input_graph;
            global graph_dimension;
            global aimnode;
            global result_dij;
            dimension=graph_dimension;
            graph_input=input_graph; %use global variables
            %initialization
            result_dij = zeros(50,dimension);
            %             D = ones(1,dimension)*inf; %distance vector
            %             D(aimnode) = 0; %the distance to the source node is 0
            result_dij(2,:)=ones(1,dimension)*inf;
            result_dij(2,aimnode)=0;
            remain_node(1:dimension)=1:dimension; %nodes that have not visited
            result_dij(1,:)=1:dimension; %the first row of result matrix is nodes' name
            %main cycle
            i=1;
            while ~isequal(remain_node,zeros(1,dimension))
                i=i+1;
                dis_temp = result_dij(i,:);
                dis_temp(find(remain_node==0))=inf; %prevent the program find visited nodes as min
                [m,I] = min(dis_temp);%m:minimum value I：the position of the value
                remain_node(I) = 0; %add this minimum node to N
                result_dij(i,I) = m; %minimum value
                C = graph_input(I,:);
                for t = 1:dimension %relaxation
                    result_dij(i,t)=min(result_dij(i,t),result_dij(i,I)+ C(t));
                    result_dij(i+1,:)=result_dij(i,:);
                end
            end
            %visualization
            fig = uifigure('Position',[100 100 752 250]);
            uit = uitable('Parent',fig,'Position',[25 50 700 200]);
            uit.ColumnName ='numbered'; %set colomn name 1->dimension
            uit.Data=result_dij(2:i,:); %load data
            uit.RowName = 'distance'; %set row name
            uit.BackgroundColor = [1 1 .9; .9 .95 1;1 .5 .5]; %set different color between columns
        end

        % Value changed function: NotesTextArea
        function NotesTextAreaValueChanged(app, event)
            value = app.NotesTextArea.Value;
            app.NotesTextArea.Value='1';
        end

        % Button pushed function: RunBellmanFordalgorithmButton
        function RunBellmanFordalgorithmButtonPushed(app, event)
            %import graph information
            global input_graph;
            global graph_dimension;
            global aimnode;
            global result_bel;
            %import graph information
            destination=aimnode;
            dimension=graph_dimension;
            %initialization
            map = [];
            show=zeros(300,dimension);
            show(1,:) = ones(1,(dimension))*inf; %the second row is the distance
            show(1,destination)=0; %the distance to itself is 0
            show(2,:) = ones(1,dimension)*-1; %the third row is next node
            show(2,destination) = destination; %the next node to the destination is still itself
            %using BFS to form the list
            vflag=zeros(graph_dimension,1);          % 初始化所有节点访问标志位
            queue=[];                        % 遍历缓存队列，每次访问并丢弃队首
            visitinglist=[];                       % 遍历结果
            startNode=aimnode;
            queue=[queue;startNode];         % 更新遍历缓存队列
            vflag(startNode)=1;              % 更新访问标志
            visitinglist=[visitinglist;startNode];       % 更新遍历结果队列
            
            while all(vflag)==0
                i=queue(1);
                queue(1)=[];
                for j=1:graph_dimension
                    if(input_graph(i,j)~=inf&&vflag(j)==0&&i~=j)
                        queue=[queue;j];
                        vflag(j)=1;
                        visitinglist=[visitinglist;j];
                    end
                end
            end
            visitinglist=visitinglist';
            flag=0;
for m = 1:dimension %setting visiting list, format: [node1; node2; weight]
    for n = 1:dimension
        if ((input_graph(visitinglist(m),n) ~= 0))
            map = [map; visitinglist(m) n input_graph(visitinglist(m),n)];
        end
    end
end
            map = map';
            V=length(map(1,:)); %the number of edges
            i=1;
            for x = 1:(V-1) %the number should be V-1
                for m=1:dimension
                    show(i+2,m)=show(i,m);
                    show(i+3,m)=show(i+1,m);
                end
                for r = 1:V
                    
                    u=map(1,r);%get data from map
                    v=map(2,r);
                    w=map(3,r);
                    if show(i,v) > (show(i,u)+w) %relaxation
                        show(i+2,v) = (show(i,u)+w);
                        show(i+3,v)= u;%minimum node
                    end
                end
                
                if (isequal(show(i+2,:),show(i,:))&&isequal(show(i+1,:),show(i+3,:)))
                    break
                end
                i=i+2;
            end
            %visualization
            fig = uifigure('Position',[100 100 752 400]);
            uit = uitable('Parent',fig,'Position',[25 50 700 300]);
            uit.ColumnName ='numbered';
            uit.Data=show(1:i+1,:);
            uit.RowName = repmat([{'distance','next node'}],1,(i+1)/2);
            uit.BackgroundColor = [1 1 .9; .9 .95 1];
            result_bel=show;
        end

        % Value changed function: NumberofnodesEditField
        function NumberofnodesEditFieldValueChanged2(app, event)
            % loading dimension from input
            global graph_dimension;
            value = app.NumberofnodesEditField.Value;%get input
            graph_dimension = value
        end

        % Callback function
        function GraphConnectionsEditFieldValueChanged3(app, event)
            %setting adjacent matrix from the input
            value = app.GraphConnectionsEditField.Value;
            graph_dimension=app.NumberofnodesEditField.Value;
            global value1;
            global input_graph;
            value1=str2num(value);
            input_graph=ones(graph_dimension)*inf;%initate matrix
            for i=1:3:length(value1) %setting matrix from the input
                input_graph(value1(i),value1(i+1))=value1(i+2);%g(i,j)
                input_graph(value1(i+1),value1(i))=value1(i+2);%g(j,i)
            end
            input_graph(logical(eye(size(input_graph))))=0;%g(i,i)=0
        end

        % Button pushed function: RunDialalgorithmButton
        function RunDialalgorithmButtonPushed(app, event)
            %Initialize
            global input_graph;
            graph_input=input_graph;
            global graph_dimension;
            dimension=graph_dimension;
            global aimnode;
            source=aimnode;
            result = zeros(2,dimension);
            buckets=zeros(10,30);%initialize buckets
            D = ones(1,dimension)*inf;%distance vector
            D(source) = 0;% the distance to itself is 0
            remain_node(1:dimension)=1:dimension;%nodes that has not visited
            result(1,:)=1:dimension;
            while ~isequal(remain_node,zeros(1,dimension))
                dis_temp = D;
                dis_temp(find(remain_node==0))=inf; %prevent from visiting visited nodes
                buckets=zeros(10,30);%clear bucket
                for i=1:dimension
                    if dis_temp(i)==inf
                        continue
                        %push in, nodes is pushed according to their distances
                    else
                        m=buckets(1,dis_temp(i)+1)+2;%read the number of bucket elements
                        buckets(m,dis_temp(i)+1)=i;%push in
                        buckets(1,dis_temp(i)+1)=buckets(1,dis_temp(i)+1)+1;%the count of element in the bucket +1
                    end
                end
                %pop out the minimum one
                Index=0;
                for i=1:30
                    if (buckets(1,i)~=0)
                        Index=buckets(2,i);%pop out
                        buckets(1,i)=buckets(1,i)-1;%the count of the bucket elements -1
                        break;
                    end
                end
                remain_node(Index) = 0;%set this node to visited
                result(2,Index) = i-1;%set the minimum distance
                C = graph_input(Index,:);
                for t = 1:dimension %relaxation
                    D(t)=min(D(t),D(Index) + C(t));
                end
            end
            %visualization
            fig = uifigure('Position',[100 100 752 250]);
            uit = uitable('Parent',fig,'Position',[25 50 700 200]);
            uit.ColumnName ='numbered';
            uit.Data=result(2,:);
            uit.RowName = 'distance';
            uit.BackgroundColor = [1 1 .9];
        end

        % Value changed function: SourceDestinationnodeEditField
        function SourceDestinationnodeEditFieldValueChanged(app, event)
            value = app.SourceDestinationnodeEditField.Value;%get input
            global aimnode;%set global variable
            aimnode=value;
        end

        % Button pushed function: ShowAdjacentMatrixButton
        function ShowAdjacentMatrixButtonPushed(app, event)
            global input_graph;
            graph_input=input_graph;
            %             save ('adjacentmatrix.mat',"graph_input","-mat");
            %visualization
            fig = uifigure('Position',[100 100 752 250]);
            uit = uitable('Parent',fig,'Position',[25 50 700 200]);
            uit.ColumnName =string(1:app.NumberofnodesEditField.Value);
            uit.Data=graph_input;
            uit.RowName = string(1:app.NumberofnodesEditField.Value);
            uit.BackgroundColor = [1 1 .9; .9 .95 1;1 .5 .5];
        end

        % Button pushed function: RunSPFAButton
        function RunSPFAButtonPushed(app, event)
            %Initialize
            global input_graph;
            graph_input=input_graph;
            global graph_dimension;
            dimension=graph_dimension;
            global aimnode;
            source=aimnode;
            queue = []; %initialize queue
            front=1; %front head
            rear=front; %queue empty
            result(1,:) = inf*ones(1,dimension); %minimum distance
            result(2,:)=source*ones(1,dimension); %next node
            result(1,source) = 0; %the distance to itself is 0
            queue(front) = source; %queue<-source
            while(front<=rear) %when the queue is not empty
                current = queue(front); %pop out from queue
                for j=1:dimension %update distances
                    if result(1,j)>result(1,current)+graph_input(current,j) %if current distance is larger than actual distance
                        result(1,j) = result(1,current)+graph_input(current,j); %update distance
                        result(2,j) = current; %set next node
                        if ~ismember (j,queue(front:rear)) %add this node to the queue (in the bottom)
                            rear = rear+1; %update pointer of the queue
                            queue(rear) = j; %write queue
                        end
                    end
                end
                front = front+1; %delete front from queue
            end
            %visualization
            fig = uifigure('Position',[100 100 752 250]);
            uit = uitable('Parent',fig,'Position',[25 50 700 200]);
            uit.ColumnName ='numbered';
            uit.Data=result;
            uit.RowName = {'distance','next node'};
            uit.BackgroundColor = [1 1 .9; .9 .95 1];
        end

        % Menu selected function: InputfromfileMenu
        function InputfromfileMenuSelected(app, event)
            global graph_dimension;
            global input_graph;
            while(1)
                flag=1;
                temp=cell2mat(struct2cell(load(uigetfile('.mat', 'Please choose input matrix file'))));
                [m,n]=size(temp);
                if ((m~=n))
                    msgbox('It must be square matrix!','FAILED');
                    flag=2;
                end
                for x=1:m
                    for y=1:n
                        if ((x==y)&&temp(x,y)~=0)
                            msgbox('Diag elements must be 0!','FAILED');
                            flag=2;
                        end
                        if (temp(x,y)<0)
                            msgbox('Elements must be above 0!','FAILED');
                            flag=2;
                        end
                        if (temp(x,y)~=temp(y,x))
                            msgbox('It must be an undirected graph','FAILED');
                            flag=2;
                        end
                    end
                end
                if (flag==1)
                    break
                end
            end
            input_graph=temp;
            graph_dimension=m;
            msgbox('Load Successfully!','Load');
        end

        % Menu selected function: LoadsampleInputMenu
        function LoadsampleInputMenuSelected(app, event)
            value=[1 2 3 1 3 2 1 4 5 3 4 2 2 4 1 2 5 4 5 6 2 3 6 1 4 5 3];
            global input_graph;
            global graph_dimension;
            graph_dimension = 6;
            input_graph=ones(graph_dimension)*inf;%initate matrix
            for i=1:3:length(value) %setting matrix from the input
                input_graph(value(i),value(i+1))=value(i+2);%g(i,j)
                input_graph(value(i+1),value(i))=value(i+2);%g(j,i)
            end
            input_graph(logical(eye(size(input_graph))))=0;%g(i,i)=0
        end

        % Menu selected function: InputmanuallyMenu
        function InputmanuallyMenuSelected(app, event)
            global graph_dimension;
            global input_graph;
            input_graph=ones(graph_dimension)*inf;
            input_graph(logical(eye(size(input_graph))))=0;%g(i,i)=0
            fig = uifigure('Position',[100 100 752 250]);
            uit = uitable('Parent',fig,'Position',[25 50 700 200]);
            uit.ColumnEditable = true;
            uit.ColumnName ='numbered';
            uit.Data=input_graph;
            uit.RowName = 'numbered';
            uit.BackgroundColor = [1 1 .9; .9 .95 1;1 .5 .5];
            uit.CellEditCallback = @autoedit;
            input_graph=uit.Data;
            function autoedit(src,eventdata)
                if (eventdata.NewData < 0 )
                    tableData = src.Data;
                    tableData(eventdata.Indices(1),eventdata.Indices(2)) = eventdata.PreviousData;
                    src.Data = tableData;                              % set the data back to its original value
                    msgbox('Cost must be above 0','warning')          % warn the user
                end
                if (eventdata.Indices(1) == eventdata.Indices(2))
                    tableData = src.Data;
                    tableData(eventdata.Indices(1),eventdata.Indices(2)) = eventdata.PreviousData;
                    src.Data = tableData;                              % set the data back to its original value
                    msgbox('Diag elements must be 0.','warning')          % warn the user
                end
                if (eventdata.Indices(1) ~= eventdata.Indices(2)&&(eventdata.NewData >=0 ))
                    tableData = src.Data;
                    tableData(eventdata.Indices(2),eventdata.Indices(1)) = eventdata.NewData;
                    src.Data = tableData;
                end
                input_graph=tableData;
            end
        end

        % Menu selected function: GeneraterandomInputMenu
        function GeneraterandomInputMenuSelected(app, event)
            global graph_dimension;
            global input_graph;
            graph_dimension=round(rand(1)*3)+5;
            A=ceil(rand(graph_dimension,graph_dimension)*9);
            for i=1:round(graph_dimension*1.8)
                A((round(rand(1)*(graph_dimension-1))+1),(round(rand(1)*(graph_dimension-1))+1))=inf;
            end
            input_graph=tril(A,-1)+triu(A',0);
            input_graph(logical(eye(size(input_graph))))=0;%g(i,i)=0
            msgbox('Generate Successfully!','Generate');
        end

        % Menu selected function: 
        % SaveDijkstraAlgorithmresulttofileMenu
        function SaveDijkstraAlgorithmresulttofileMenuSelected(app, event)
            global result_dij;
            save ('dijkstra_result.mat',"result_dij",'-mat');
            msgbox('result save to file dijkstra_result.mat','Save');
        end

        % Menu selected function: 
        % SaveBellmanFordAlgorithmresulttofileMenu
        function SaveBellmanFordAlgorithmresulttofileMenuSelected(app, event)
            
            global result_bel;
            save ('Bellman-Ford_result.mat',"result_bel",'-mat');
            msgbox('result saved to file Bellman-Ford_result.mat','Save');
        end

        % Menu selected function: SaveAdjacentMatrixtofileMenu
        function SaveAdjacentMatrixtofileMenuSelected(app, event)
            global input_graph;
            save ('input_graph.mat',"input_graph",'-mat');
            msgbox('result saved to file input_graph.mat','Save');
        end
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Create TheShortestPathAlgorithmUIFigure and hide until all components are created
            app.TheShortestPathAlgorithmUIFigure = uifigure('Visible', 'off');
            app.TheShortestPathAlgorithmUIFigure.Position = [100 100 640 480];
            app.TheShortestPathAlgorithmUIFigure.Name = 'The Shortest Path Algorithm';
            app.TheShortestPathAlgorithmUIFigure.Scrollable = 'on';

            % Create InputMenu
            app.InputMenu = uimenu(app.TheShortestPathAlgorithmUIFigure);
            app.InputMenu.Text = 'Input';

            % Create InputfromfileMenu
            app.InputfromfileMenu = uimenu(app.InputMenu);
            app.InputfromfileMenu.MenuSelectedFcn = createCallbackFcn(app, @InputfromfileMenuSelected, true);
            app.InputfromfileMenu.Text = 'Input from file';

            % Create InputmanuallyMenu
            app.InputmanuallyMenu = uimenu(app.InputMenu);
            app.InputmanuallyMenu.MenuSelectedFcn = createCallbackFcn(app, @InputmanuallyMenuSelected, true);
            app.InputmanuallyMenu.Text = 'Input manually';

            % Create GeneraterandomInputMenu
            app.GeneraterandomInputMenu = uimenu(app.InputMenu);
            app.GeneraterandomInputMenu.MenuSelectedFcn = createCallbackFcn(app, @GeneraterandomInputMenuSelected, true);
            app.GeneraterandomInputMenu.Text = 'Generate random Input';

            % Create LoadsampleInputMenu
            app.LoadsampleInputMenu = uimenu(app.InputMenu);
            app.LoadsampleInputMenu.MenuSelectedFcn = createCallbackFcn(app, @LoadsampleInputMenuSelected, true);
            app.LoadsampleInputMenu.Text = 'Load sample Input';

            % Create SaveMenu
            app.SaveMenu = uimenu(app.TheShortestPathAlgorithmUIFigure);
            app.SaveMenu.Text = 'Save';

            % Create SaveDijkstraAlgorithmresulttofileMenu
            app.SaveDijkstraAlgorithmresulttofileMenu = uimenu(app.SaveMenu);
            app.SaveDijkstraAlgorithmresulttofileMenu.MenuSelectedFcn = createCallbackFcn(app, @SaveDijkstraAlgorithmresulttofileMenuSelected, true);
            app.SaveDijkstraAlgorithmresulttofileMenu.Text = 'Save Dijkstra Algorithm result to file ';

            % Create SaveBellmanFordAlgorithmresulttofileMenu
            app.SaveBellmanFordAlgorithmresulttofileMenu = uimenu(app.SaveMenu);
            app.SaveBellmanFordAlgorithmresulttofileMenu.MenuSelectedFcn = createCallbackFcn(app, @SaveBellmanFordAlgorithmresulttofileMenuSelected, true);
            app.SaveBellmanFordAlgorithmresulttofileMenu.Text = 'Save Bellman-Ford Algorithm result to file ';

            % Create SaveAdjacentMatrixtofileMenu
            app.SaveAdjacentMatrixtofileMenu = uimenu(app.SaveMenu);
            app.SaveAdjacentMatrixtofileMenu.MenuSelectedFcn = createCallbackFcn(app, @SaveAdjacentMatrixtofileMenuSelected, true);
            app.SaveAdjacentMatrixtofileMenu.Text = 'Save Adjacent Matrix to file ';

            % Create RunDijkstraalgorithmButton
            app.RunDijkstraalgorithmButton = uibutton(app.TheShortestPathAlgorithmUIFigure, 'push');
            app.RunDijkstraalgorithmButton.ButtonPushedFcn = createCallbackFcn(app, @RunDijkstraalgorithmButtonPushed, true);
            app.RunDijkstraalgorithmButton.Position = [24 153 154 64];
            app.RunDijkstraalgorithmButton.Text = 'Run Dijkstra algorithm';

            % Create NotesTextAreaLabel
            app.NotesTextAreaLabel = uilabel(app.TheShortestPathAlgorithmUIFigure);
            app.NotesTextAreaLabel.HorizontalAlignment = 'right';
            app.NotesTextAreaLabel.Position = [379 449 45 22];
            app.NotesTextAreaLabel.Text = 'Notes:';

            % Create NotesTextArea
            app.NotesTextArea = uitextarea(app.TheShortestPathAlgorithmUIFigure);
            app.NotesTextArea.ValueChangedFcn = createCallbackFcn(app, @NotesTextAreaValueChanged, true);
            app.NotesTextArea.Editable = 'off';
            app.NotesTextArea.Position = [379 153 247 286];
            app.NotesTextArea.Value = {'1. All nodes will be represented as continous numbers and start from 1.'; ''; '2. If you choose to use ''Input manually'' from the Input menu, please input number of nodes first.'; ''; '3. If you choose to input from file, the input file must in the same directory with the program. '; ''; '4. If you want to input from file, the file should be *.mat. '; ''; '5. You only need to input number of nodes when you choose ''input manully''.'; ''; '6. Data input manually will be automatically filled to the other half of the matrix. '};

            % Create Image
            app.Image = uiimage(app.TheShortestPathAlgorithmUIFigure);
            app.Image.Position = [413 25 213 126];
            app.Image.ImageSource = '-1.png';

            % Create RunBellmanFordalgorithmButton
            app.RunBellmanFordalgorithmButton = uibutton(app.TheShortestPathAlgorithmUIFigure, 'push');
            app.RunBellmanFordalgorithmButton.ButtonPushedFcn = createCallbackFcn(app, @RunBellmanFordalgorithmButtonPushed, true);
            app.RunBellmanFordalgorithmButton.Position = [24 65 154 64];
            app.RunBellmanFordalgorithmButton.Text = 'Run Bellman-Ford algorithm';

            % Create ShowNetworkTopologyButton
            app.ShowNetworkTopologyButton = uibutton(app.TheShortestPathAlgorithmUIFigure, 'push');
            app.ShowNetworkTopologyButton.ButtonPushedFcn = createCallbackFcn(app, @ShowNetworkTopologyButtonPushed, true);
            app.ShowNetworkTopologyButton.Position = [24 242 154 64];
            app.ShowNetworkTopologyButton.Text = 'Show Network Topology';

            % Create NumberofnodesEditFieldLabel
            app.NumberofnodesEditFieldLabel = uilabel(app.TheShortestPathAlgorithmUIFigure);
            app.NumberofnodesEditFieldLabel.HorizontalAlignment = 'right';
            app.NumberofnodesEditFieldLabel.Position = [21 399 106 22];
            app.NumberofnodesEditFieldLabel.Text = 'Number of nodes';

            % Create NumberofnodesEditField
            app.NumberofnodesEditField = uieditfield(app.TheShortestPathAlgorithmUIFigure, 'numeric');
            app.NumberofnodesEditField.ValueChangedFcn = createCallbackFcn(app, @NumberofnodesEditFieldValueChanged2, true);
            app.NumberofnodesEditField.Position = [173 399 56 22];

            % Create RunDialalgorithmButton
            app.RunDialalgorithmButton = uibutton(app.TheShortestPathAlgorithmUIFigure, 'push');
            app.RunDialalgorithmButton.ButtonPushedFcn = createCallbackFcn(app, @RunDialalgorithmButtonPushed, true);
            app.RunDialalgorithmButton.Position = [204 153 154 64];
            app.RunDialalgorithmButton.Text = 'Run Dial algorithm';

            % Create RunSPFAButton
            app.RunSPFAButton = uibutton(app.TheShortestPathAlgorithmUIFigure, 'push');
            app.RunSPFAButton.ButtonPushedFcn = createCallbackFcn(app, @RunSPFAButtonPushed, true);
            app.RunSPFAButton.Position = [204 65 154 64];
            app.RunSPFAButton.Text = 'Run SPFA';

            % Create ShowAdjacentMatrixButton
            app.ShowAdjacentMatrixButton = uibutton(app.TheShortestPathAlgorithmUIFigure, 'push');
            app.ShowAdjacentMatrixButton.ButtonPushedFcn = createCallbackFcn(app, @ShowAdjacentMatrixButtonPushed, true);
            app.ShowAdjacentMatrixButton.Position = [204 242 154 64];
            app.ShowAdjacentMatrixButton.Text = 'Show Adjacent Matrix';

            % Create SourceDestinationnodeEditFieldLabel
            app.SourceDestinationnodeEditFieldLabel = uilabel(app.TheShortestPathAlgorithmUIFigure);
            app.SourceDestinationnodeEditFieldLabel.HorizontalAlignment = 'right';
            app.SourceDestinationnodeEditFieldLabel.Position = [9 349 151 22];
            app.SourceDestinationnodeEditFieldLabel.Text = 'Source/Destination node';

            % Create SourceDestinationnodeEditField
            app.SourceDestinationnodeEditField = uieditfield(app.TheShortestPathAlgorithmUIFigure, 'numeric');
            app.SourceDestinationnodeEditField.ValueChangedFcn = createCallbackFcn(app, @SourceDestinationnodeEditFieldValueChanged, true);
            app.SourceDestinationnodeEditField.Position = [173 349 56 22];

            % Create SamplegraphLabel
            app.SamplegraphLabel = uilabel(app.TheShortestPathAlgorithmUIFigure);
            app.SamplegraphLabel.Position = [463 25 80 22];
            app.SamplegraphLabel.Text = 'Sample graph';

            % Show the figure after all components are created
            app.TheShortestPathAlgorithmUIFigure.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = The_shortest_path_algorithm

            runningApp = getRunningApp(app);

            % Check for running singleton app
            if isempty(runningApp)

                % Create UIFigure and components
                createComponents(app)

                % Register the app with App Designer
                registerApp(app, app.TheShortestPathAlgorithmUIFigure)
            else

                % Focus the running singleton app
                figure(runningApp.TheShortestPathAlgorithmUIFigure)

                app = runningApp;
            end

            if nargout == 0
                clear app
            end
        end

        % Code that executes before app deletion
        function delete(app)

            % Delete UIFigure when app is deleted
            delete(app.TheShortestPathAlgorithmUIFigure)
        end
    end
end
