%gt 167 180
%%%%%%%%%%%%%%%%%%%%%%%%%% SCENARIO 1 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
cut_off_range=3.7;
avg_array=[];
height_scene=all_scene_1;
for i=1:(length(height_scene(:,1)))
    if(height_scene(i,7)>cut_off_range)
        avg_array(end+1)=height_scene(i,9);
    end
end
mean_height_p1_s1=mean(avg_array)
cov_height_p1_s1=cov(avg_array)

avg_array=[];
for i=1:(length(height_scene(:,1)))
    if(height_scene(i,18)>cut_off_range)
        avg_array(end+1)=height_scene(i,20);
    end
end
mean_height_p2_s1=mean(avg_array)
cov_height_p2_s1=cov(avg_array)

%%%%%%%%%%%%%%%%%%%%%%%%%% SCENARIO 2 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
cut_off_range=3.7;
avg_array=[];
height_scene=all_scene_2;
for i=1:(length(height_scene(:,1)))
    if(height_scene(i,7)>cut_off_range)
        avg_array(end+1)=height_scene(i,9);
    end
end
mean_height_p1_s2=mean(avg_array)
cov_height_p1_s2=cov(avg_array)

avg_array=[];
for i=1:(length(height_scene(:,1)))
    if(height_scene(i,18)>cut_off_range)
        avg_array(end+1)=height_scene(i,20);
    end
end
mean_height_p2_s2=mean(avg_array)
cov_height_p2_s2=cov(avg_array)

%%%%%%%%%%%%%%%%%%%%%%%%%% SCENARIO 3 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
cut_off_range=3.7;
avg_array=[];
height_scene=all_scene_3;
for i=1:(length(height_scene(:,1)))
    if(height_scene(i,7)>cut_off_range)
        avg_array(end+1)=height_scene(i,9);
    end
end
mean_height_p1_s3=mean(avg_array)
cov_height_p1_s3=cov(avg_array)

avg_array=[];
for i=1:(length(height_scene(:,1)))
    if(height_scene(i,18)>cut_off_range)
        avg_array(end+1)=height_scene(i,20);
    end
end
mean_height_p2_s3=mean(avg_array)
cov_height_p2_s3=cov(avg_array)


