classdef Mesh < handle
    %MESH  :  3D网格对象
    %继承handel后可以在对象内部修改对象的属性
    properties
        V%点  nv*3
        E%线  ne*3
        F%面  nf*3
        NV%顶点法线
        NF%平面法线
        file_name%文件名称
        matrix0%操作过程中进行的变换
        
        print%控制 网格更新后是否以绘制出来
    end
    properties (Constant,Hidden)
      m34=[1 0 0 0;0 1 0 0;0 0 1 0]
      voxel_size=0.1;%体素方块大小
    end
    methods
        function n=nv(o)
            n=size(o.V,1);
        end
        function n=ne(o)
            n=size(o.E,1);
        end
        function n=nf(o)
            n=size(o.F,1);
        end
        function out=box(o,type)
            box0=[min(o.V);max(o.V)];
            if nargin==1
                out=box0;
            else
                if type=="max"
                    out=box0(1,:);
                elseif type=="min"
                    out=box0(2,:);
                else%size
                    out=box0(2,:)-box0(1,:);
                end
            end
        end
        function o = Mesh(file_name)
            o.file_name=file_name;
            o.matrix0=eye(4);
            [o.V,o.F] = o.read(file_name);
            %o.mergeVertex();
            o.computeNormal();%计算所有平面的法线
            o.computeEdge();
            o.print=0;%不展示每次网格更新的结果
        end
        function download(o)
            %inv(o.matrix0);
            o.write(o.file_name+"_save",o.V,o.F);
        end
        
        function applyMove(o,in1,in2,in3)
            if nargin==2
                d=in1;
            else
                d=[in1,in2,in3];
            end
            o.applyMatrix([1 0 0 d(1);0 1 0 d(2);0 0 1 d(3);0 0 0 1]);
        end
        function applyRotation(o,in1,in2,in3)
            if nargin==2
                in=in1;
            else
                in=[in1,in2,in3];
            end
            o.applyMatrix(rotx(in(1))*roty(in(2))*rotz(in(3)));
        end
        function applyScale(o,in1,in2,in3)
            if nargin==2
                s=in1;
            else
                s=[in1,in2,in3];
            end
            o.applyMatrix([s(1) 0 0;0 s(2) 0;0 0 s(3)]);
        end
        function applyMatrix(o,mat)
            if length(mat)==3
                mat=[mat;[0 0 0]];
                mat=[mat';[0 0 0 1]]';
            end
            V2=[o.V';ones(o.nv,1)']';%nv*4
            o.V=(o.m34*mat*(V2'))';
            o.matrix0=mat*o.matrix0;%记录进行的变换
            if o.print==1
                o.draw();
            end
        end
        function normal(o)
            %旋转
            voxel=o.voxelization();
            mean0 = mean(voxel);% 样本均值
            Z = voxel-repmat(mean0,length(voxel), 1);%减去均值
            covMat = Z' * Z;% covMat 协方差矩阵 %r*3 3*r
            [mat,~] = eigs(covMat, 3);% V每一列为一个特征向量
            o.applyMatrix(mat');%对齐坐标轴
            
            %翻转
            voxel=o.voxelization();
            mean0 = mean(voxel);% 样本均值
            Z = voxel-repmat(mean0,length(voxel), 1);%减去均值
            o.applyScale((sum(Z.^3)>0)*2-1);%取x^3大于0的方向为正方向
            
            %移动
            voxel=o.voxelization();
            mean0 = mean(voxel);% 样本均值
            o.applyMove(mean0.*-1);
            
            
            %放缩
            %voxel=o.voxelization();
            %d=sum((voxel-mean(voxel)).^2);
            %d=sum(voxel.^2);
            %o.applyScale(ones(1,3)./(d.^0.5));%除以标准差
            
        end
        function reset(o)
            o.applyMatrix(inv(o.matrix0));
        end
        function draw(o)
            clf
            trimesh(o.F, o.V(:,1), o.V(:,2), o.V(:,3),'LineWidth',1,'EdgeColor','k');
            axis equal
            %axis off %隐藏坐标轴
            camlight
            lighting gouraud
            cameratoolbar%创建一个工具栏
            drawnow
            o.print=1;%展示每次网格更新的结果
        end
        function simplify(o,r)
            myQEM=QEM();
            o=myQEM.simplification(o,r);
            if o.print==1
                o.draw();
            end
        end
    end%methods
    methods(Hidden)
        function process(this)
            myQEM=QEM();
            this=myQEM.simplification(this,0.5);
            this.download();
        end
        function computeNormal(o)
            %输入： vertex：nv*3   face:nf*3
            %输出：
            % compute_normal - compute the normal of a triangulation
            %
            %   [normal,normalf] = compute_normal(vertex,face);
            %
            %   normal(i,:) is the normal at vertex i.
            %   normalf(j,:) is the normal at face j.
            
            
            [vertex,face] = check_face_vertex(o.V,o.F);
            %vertex：3*nv   face:3*nf
            
            nface = size(face,2);
            nvert = size(vertex,2);
            
            % unit normals to the faces 单位面法线
            normalf = crossp( vertex(:,face(2,:))-vertex(:,face(1,:)), ...
                vertex(:,face(3,:))-vertex(:,face(1,:)) );
            d = sqrt( sum(normalf.^2,1) );
            d(d<eps)=1;%eps是极小值
            normalf = normalf ./ repmat( d, 3,1 );%面法线单位化
            
            % unit normal to the vertex
            normal = zeros(3,nvert);%顶点法线 normal:3*nv
            for i=1:nface
                f = face(:,i);
                for j=1:3
                    normal(:,f(j)) = normal(:,f(j)) + normalf(:,i);
                end
            end
            % normalize
            d = sqrt( sum(normal.^2,1) ); d(d<eps)=1;
            normal = normal ./ repmat( d, 3,1 );
            
            % enforce that the normal are outward
            v = vertex - repmat(mean(vertex,1), 3,1);
            s = sum( v.*normal, 2 );
            if sum(s>0)<sum(s<0)
                % flip
                normal = -normal;
                normalf = -normalf;
            end
            o.NV=normal';
            o.NF=normalf';
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            function z = crossp(x,y)%x,y可以看做三角形的两个边，z是和它们垂直的方向
                % x and y are (m,3) dimensional
                z = x;
                z(1,:) = x(2,:).*y(3,:) - x(3,:).*y(2,:);
                z(2,:) = x(3,:).*y(1,:) - x(1,:).*y(3,:);
                z(3,:) = x(1,:).*y(2,:) - x(2,:).*y(1,:);
            end
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            function [vertex,face] = check_face_vertex(vertex,face)
                
                % check_face_vertex - check that vertices and faces have the correct size
                %
                %   [vertex,face] = check_face_vertex(vertex,face);
                %
                %   Copyright (c) 2007 Gabriel Peyre
                
                vertex = check_size(vertex,2,4);
                face = check_size(face,3,4);
            end
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            function a = check_size(a,vmin,vmax)
                if isempty(a)
                    return;
                end
                if size(a,1)>size(a,2)
                    a = a';
                end
                if size(a,1)<3 && size(a,2)==3
                    a = a';
                end
                if size(a,1)<=3 && size(a,2)>=3 && sum(abs(a(:,3)))==0
                    % for flat triangles
                    a = a';
                end
                if size(a,1)<vmin ||  size(a,1)>vmax
                    error('face or vertex is not of correct size');
                end
            end
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end%函数结束
        function computeEdge(o)
            TR = triangulation(o.F,o.V);%进行三角剖分，梳理出所有三角形
            o.E = edges(TR);%返回所有边的顶点索引  ne*2
        end
        function rectifyindex(o)
            %清除为空的顶点
            %RECTIFYINDEX Summary of this function goes here
            
            
            num_of_NaN=zeros(o.nv(),1);
            sum=0;
            for i=1:o.nv()
                if isnan(o.V(i,1)) % 为空 NaN
                    sum=sum+1;
                end
                num_of_NaN(i)=sum;
            end
            
            recF=zeros(o.nf(),3);%三角面个数不变，但是由于顶点个改变，三角面的顶点索引需要修改
            for i=1:o.nf()
                for j=1:3
                    recF(i,j)=o.F(i,j)-num_of_NaN(o.F(i,j));
                end
            end
            
            recV=zeros(o.nv-sum,3);%总个数-为空的个数
            j=1;
            for i=1:o.nv()
                if ~isnan(o.V(i,1))
                    recV(j,:)=o.V(i,:);
                    j=j+1;
                end
            end
            o.V=recV;
            o.F=recF;
        end
        function voxel=voxelization(o)
            box=o.box();
            box_size=box(2,:)-box(1,:);
            step=min(box_size)*o.voxel_size;%体素方块大小
            if step==0
                step=1;
            end
            %box_size_step=ceil(box_size./step);
            voxel=[];%zeros(box_size_step);
            for i=1:o.nf()
                oF=o.F();
                v1=o.V(oF(i,1),:);
                v2=o.V(oF(i,2),:);
                v3=o.V(oF(i,3),:);
                voxel=addF(v1,v2,v3,voxel,box,step);
            end
            voxel=voxel.*o.voxel_size;%+repmat(box(1,:),length(voxel),1);%体素方块大小
            function voxel=addF(v1,v2,v3,voxel,box,step)
                for A=0:0.2:1
                    for B=0:0.2:(1-A)
                        v=v1.*A+v2.*B+v3.*(1-A-B);
                        voxel=addV(v,voxel,box,step);
                    end
                end
            end
            function voxel=addV(v,voxel,box,step)
                v=v-box(1,:);
                v=round(v./step);
                %v=v+ones(size(v));
                if v(1)==0
                    v(1)=1;
                end
                if v(2)==0
                    v(2)=1;
                end
                if v(3)==0
                    v(3)=1;
                end
                voxel=[voxel;[v(1),v(2),v(3)]];
                %voxel(v(1),v(2),v(3))=1;
            end
        end
    end%methods(Hidden)
    methods(Static)
        function [vertex,faces] = read(filename)
            vertex = [];
            faces = [];
            fid = fopen(filename+".obj");%fid是一个大于0的整数
            s = fgetl(fid);
            while ischar(s)
                if ~isempty(s)
                    if strcmp(s(1), 'f')%如果字符串第一个字符为f %face
                        %  F V1 V2 V3 ...
                        %  F V1/VT1/VN1  ...
                        %  F V1//VN1  ...
                        str2=strsplit(s," ");
                        coordinate1=strsplit(cell2mat(str2(2)),"/");
                        if isempty(length(coordinate1)==1)
                            faces(end+1,:) =sscanf(s(3:end), '%d %d %d');
                        else
                            coordinate2=strsplit(cell2mat(str2(3)),"/");%length(str2num("1 "))
                            coordinate3=strsplit(cell2mat(str2(4)),"/");
                            faces(end+1,:) = [
                                str2double(coordinate1(1))
                                str2double(coordinate2(1))
                                str2double(coordinate3(1))
                                ];
                        end
                    elseif strcmp(s(1), 'v')%如果字符串第一个字符为v %vertex
                        vertex(end+1,:) = sscanf(s(3:end), '%f %f %f');
                    end%vertex添加一行、在最后一列  s从第三个字符开始到最后一个
                end
                s = fgetl(fid);%获取下一行
            end
            fclose(fid);
        end
        function write(filename,vertices,faces )
            fid=fopen(filename+".obj",'w');
            fid=arrPrintf(fid,vertices,'v');
            fid=arrPrintf(fid,faces,'f');
            fclose(fid);
            function fid=arrPrintf(fid,arr,head)
                [x,y]=size(arr);
                for i=1:x
                    fprintf(fid,head);
                    for j=1:y
                        fprintf(fid,' %d',arr(i,j));
                    end
                    fprintf(fid,'\n');%每一行回车\n  
                end
            end
        end
    end%methods(Static)
    methods(Static,Hidden)%用于测试的方法
        function test()
            mesh=Mesh("man_sim2");
            myQEM=QEM();
            mesh=myQEM.simplification(mesh,0.5);
            mesh.download();
        end
        function test2()
            mesh=Mesh("mesh2");
            mesh.box();
            mesh.applyMatrix([
                1 0 0 ;
                0 0 1 ;
                0 1 0 
                ]);
            voxel=mesh.voxelization();
            size(voxel);
            %sum(voxel,"all")
            mesh.normal();
            mesh.draw();
            %mesh.download();
        end
        function test3()
            %untitled
            Mesh.read("untitled");
        end
    end%methods(Static)
end%class

