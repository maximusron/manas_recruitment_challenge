template <class Q>
void clearQueue(Q& q){
    q = Q();
}

namespace globalplanner{
//// Global functions in globalplanner namespace:
double to_rads(const double& angle){
    return (angle*M_PI)/180.0;
}
int to_deg(const double& angle){
    return (angle*180)/M_PI;
}

int correct_deg(int angle){
    int a=angle;
    if(angle>=360)
        a = angle%360;
    else if(angle<0)
        a = 360 + angle;
    return a;
}

double deg_to_rads_inrange(int angle){
    int a = correct_deg(angle);
    if(a>=180)
        return to_rads(a-360);
    else
        return to_rads(a);
}

vector<double> find_vel(const int& time_frame,const float& d,const int& x, const int& y, const int& angle, const int& x_prev,const int& y_prev, int angle_prev){
    vector<double> vel;
    double theta = to_rads(angle);
    double theta_prev = to_rads(angle_prev);
    double w = (theta-theta_prev)/time_frame;
    double eqn2 = d*w; // Vr-Vl
    double vx = ((x-x_prev)*w)/sin(w*time_frame);
    double vy = ((y-y_prev)*w)/(1-cos(w*time_frame));
    double eqn1 = (vx+vy); //just to take avg to reduce diff
    double Vr = (eqn1 + eqn2)/2;
    double Vl = (eqn1 - eqn2)/2;
    vel.clear();
    vel.push_back(Vl);
    vel.push_back(Vr);
    return vel;
}



//// PlannerAlgorithm methods:

vector<int> PlannerAlgorithm::findpath(const vector<Node>& matrix, const int& dest){
    // cout<<"the path:"<<endl;
    //ROS_INFO("checkpoint findpath 1");
    vector<int> path;
    
    cout<<"findpath pathlen "<<matrix[dest].g<<endl;
    int next_node = matrix[dest].parent;
    path.push_back(dest);
    do{
        path.push_back(next_node);
        next_node = matrix[next_node].parent;
    }while(matrix[next_node].parent!=next_node);

    path.push_back(next_node);
    
    // for(auto i=path.rbegin();i!=path.rend();++i){
    //     cout<<"->"<<(*i)/width<<","<<(*i)%width;

    return path;
}

vector<int> PlannerAlgorithm::findNeighbours(int index){
    int row = index/width;
    int col = index%width;
    vector<int> neighbours;

    for(int add_x=-1; add_x<=1;add_x++)
    {
        for(int add_y=-1; add_y<=1;add_y++)
        {	//ROS_INFO("checkpoint astar while 2");
            if(!(isInside(row+add_y,col+add_x) &&(!(add_x==0 && add_y==0)))) continue;
            //if(!(add_x*add_y == 0)) continue; // if robot can only tranverse in 4 directions 
            int n = index + add_y*width + add_x;
            if(isNotBlocked(n)) neighbours.push_back(n);
        }
    }
    return neighbours;
}

bool PlannerAlgorithm::isInside(const int& row, const int& col){ //input in (i,j) or (row,col)
    return (col >= 0)&&(col < width)&&(row >=0)&&(row < height);
}

double PlannerAlgorithm::distance(const int& src, const int& dest){
    return sqrt(pow((src/width)-(dest/width),2.0) + pow((src%width) - (dest%width),2.0));
}

double PlannerAlgorithm::distance_squared(const int& src, const int& dest){ // reduce computation
    return (pow((src/width)-(dest/width),2.0) + pow((src%width) - (dest%width),2.0));
}

double PlannerAlgorithm::diagonalDist(const int& src, const int& dest){
    int dx = abs(src%width - dest%width);
    int dy = abs(src/width - dest/width);
    double D = 1.0, D2 = 1.4142; // D and D2 straight and diagonal dist. configure to modify heuristc 
    return (D * (dx + dy) + (D2 - 2 * D) * min(dx, dy));
}

double PlannerAlgorithm::manhattanDist(const int& src, const int& dest){
    int dx = abs(src%width - dest%width);
    int dy = abs(src%width - dest%width);
    int D = 1;
    return D * (dx+dy);
}

double PlannerAlgorithm::heuristic(const int& src, const int& dest){
    return distance(src,dest);
}

bool PlannerAlgorithm::isValid(const int& p){
    //return (p.x>=0) && (p.x<row) && (p.y>=0) && (p.y<col);
    return true;
}

bool PlannerAlgorithm::isNotBlocked(const int& p){
    //for binary
    //ROS_INFO("isNotBlocked");
    return isValid(p) && (253>costmap_->getCost(p%width,p/width));
    //for costmap
    //return isValid(map,p);
    //return true; // for the time being. we need both x&y for this
}

double PlannerAlgorithm::kernel_cost(int index, int ksize){
    int k = (ksize-1)/2;
    double cost = 0;
    int row = index/width, col = index%width;
    for(int i = row-k; i<=row+k;i++){
        for(int j=col-k;j<=col+k;j++){
            if(ksize==1 || isInside(i,j))
                cost += costmap_->getCost(j,i); //pass in (x,y format). 
        }
    }
    return cost;
}

double PlannerAlgorithm::avg_kernel_cost(int index, int ksize){
    double cost = kernel_cost(index,ksize);
    return cost/(ksize*ksize);
}

///// Astar_smooth methods:

vector<Index> Astar_smooth::search_path(const Index& start_pt, const int& dest,const double& accn,vector<double> initial_vel,const int& time_frame,const double& d_robot){
    //auto start_clock1 = std::chrono::steady_clock::now();

    vector<Index> path;
    int src = start_pt.pos;
    //ROS_INFO("checkpoint astar 1");
    if (!isValid(src)){
        cout<<"source is invalid"<<endl;
        return path;
    }
    if (!isValid(dest)){
        cout<<"dest is invalid"<<endl;
        return path;
    }
    if (!isNotBlocked(dest)){
        cout<<"dest blocked"<<endl;
        return path;
    }
    if (src==dest){
        cout<<"source and dest sameee"<<endl;
        return path;
    }

    //bool visited[maplen]; //already visited/explored cells
    //memset(visited,false,sizeof(visited));
    vector<vector<bool>> visited3d(maplen,vector<bool>(360,false));
    //ROS_INFO("checkpoint astar 2");
    
    //array<Node,maplen> matrix;
    //Node matrix[maplen];
    vector<vector<Node_big>> matrix3d(maplen,vector<Node_big>(360));
    int i = src;
    int theta = start_pt.orientation;
    matrix3d[i][start_pt.orientation].f = 0.0;
    matrix3d[i][start_pt.orientation].g = 0.0;
    //matrix[i].h = 0.0;
    matrix3d[i][start_pt.orientation].parent.pos = i;
    matrix3d[i][start_pt.orientation].parent.orientation = start_pt.orientation;
    matrix3d[i][theta].vel_l = initial_vel[0];
    matrix3d[i][theta].vel_r = initial_vel[1];
    //ROS_INFO("checkpoint astar 3");
    //passing (f,x,y) in priority queue. min_f at top
    //toExplore is to be explored nodes/cells
    std::priority_queue<Tuple, std::vector<Tuple>, std::greater<Tuple>> toExplore;
    toExplore.emplace(0.0,i,theta);
    //ROS_INFO("checkpoint astar 4");

    //int early_successor = -1;
    while (!toExplore.empty())
    {   //auto end_clockpq = std::chrono::steady_clock::now();
        //if(early_successor > -1) i = early_successor;
    
        const Tuple& p = toExplore.top();
        i = get<1>(p);
        theta = get<2>(p);
        toExplore.pop();
        
	    //auto start_clockpq = std::chrono::steady_clock::now();
	    //auto diff_timepq = start_clockpq - end_clockpq;
        //cout << std::chrono::duration <double, milli> (diff_timepq).count() << " ms for pq" << endl;

        if (visited3d[i][theta] == true) continue; //to remove duplicate longer nodes
        visited3d[i][theta] = true;
        //ROS_INFO("checkpoint astar while 1");
	    if(matrix3d[i][theta].g>30000){ // won't generate full path but small goal faster
	        path = findpath(matrix3d,i);
	        ROS_INFO("not finding full path. uncomment");
	    return path;
	    }
	//ROS_INFO("checkpoint astar while 2");
        //early_successor = -1;
        //double lowest_neighbour = get<0>(toExplore.top());

        vector<Neighbour_smooth> neighbours = findNeighbours(i,theta,accn,matrix3d[i][theta].vel_l,matrix3d[i][theta].vel_r,time_frame,d_robot);
        //cout<<"neighbour size: "<<neighbours.size()<<endl;
        for(auto loopvar=neighbours.rbegin();loopvar!=neighbours.rend();++loopvar){
            int neighbour_i = loopvar->pos;
            int neighbour_theta = loopvar->orientation;
            //ROS_INFO("pos %d orient %d  d %f c %f vl %f vr %f",loopvar->pos,loopvar->orientation,loopvar->d,loopvar->c,loopvar->V_l,loopvar->V_r);
            if(isNotBlocked(neighbour_i) && !visited3d[neighbour_i][neighbour_theta])
            {   //ROS_INFO("checkpoint astar while 3");
                if(neighbour_i==dest)
                {	//ROS_INFO("calling findpath");
                    matrix3d[neighbour_i][0].parent.pos = i;
                    matrix3d[neighbour_i][0].parent.orientation = theta;
		            matrix3d[neighbour_i][0].g = matrix3d[i][theta].g + loopvar->d;
		            //cout<<"neighbours g = "<< matrix[i].g<<endl;
                    // cout<<"dest found"<<endl;
                    path.clear();

		            //auto end_clock1 = std::chrono::steady_clock::now();
		            //auto diff_time = end_clock1 - start_clock1;
        	        //cout << std::chrono::duration <double, milli> (diff_time).count() << " ms for astar" << endl;

                    path = findpath(matrix3d,dest);

		            //start_clock1 = std::chrono::steady_clock::now();
		            //diff_time = start_clock1 - end_clock1;
        	        //cout << std::chrono::duration <double, milli> (diff_time).count() << " ms for findpath" << endl;

                    return path; 
                }

                else
                {	//ROS_INFO("adding neighbours to be explored");
                    double g_new,h_new,f_new;
                    g_new = matrix3d[i][theta].g + loopvar->d; //1 for diagonal?
                    h_new = heuristic(neighbour_i,dest);
                    f_new = loopvar->c + g_new + (g_new<h_new? 1.0000001 : 1.001)*h_new + avg_kernel_cost(neighbour_i,1);

                    //if(matrix[neighbour].f == -1 || matrix[neighbour].f > f_new)
		            if(matrix3d[neighbour_i][neighbour_theta].f > f_new) //when default is infinity (check .h file)
                    {   
			            matrix3d[neighbour_i][neighbour_theta].g = g_new;
                        //matrix[neighbour].h = h_new;  //removed from Node definition
                        matrix3d[neighbour_i][neighbour_theta].f = f_new;
                        matrix3d[neighbour_i][neighbour_theta].parent.pos = i;
                        matrix3d[neighbour_i][neighbour_theta].parent.orientation = theta;
                        matrix3d[neighbour_i][neighbour_theta].vel_l = loopvar->V_l;
                        matrix3d[neighbour_i][neighbour_theta].vel_r = loopvar->V_r;

			            
                        toExplore.emplace(matrix3d[neighbour_i][neighbour_theta].f,neighbour_i,neighbour_theta);
                    }
                }
                    
            }
        }
        
    }
    cout<<"dest not found"<<endl;
    

}

vector<Index> Astar_smooth::findpath(const vector<vector<Node_big>>& matrix, const int& dest){
    // cout<<"the path:"<<endl;
    //ROS_INFO("checkpoint findpath 1");
    vector<Index> path;
    
    cout<<"findpath pathlen "<<matrix[dest][0].g<<endl;
    Index next_node;
    next_node.pos = dest;
    next_node.orientation = 0;
    path.push_back(next_node);
    next_node.pos = matrix[dest][0].parent.pos;
    next_node.orientation = matrix[dest][0].parent.orientation;
    do{
        path.push_back(next_node);
        int i = matrix[next_node.pos][next_node.orientation].parent.pos;
        int theta = matrix[next_node.pos][next_node.orientation].parent.orientation;
        next_node.pos = i;
        next_node.orientation = theta;
    }while(matrix[next_node.pos][next_node.orientation].parent.pos!=next_node.pos && matrix[next_node.pos][next_node.orientation].parent.orientation!=next_node.orientation);

    path.push_back(next_node);
    
    // for(auto i=path.rbegin();i!=path.rend();++i){
    //     cout<<"->"<<(*i)/width<<","<<(*i)%width;

    return path;
}

vector<Neighbour_smooth> Astar_smooth::findNeighbours(const int& index,const int& angle,const double& accn,const double& U_l,const double& U_r,const int& time_frame,const double& d_robot){
    int Yi = index/width;
    int Xi = index%width;
    double theta_i = to_rads(angle);
    vector<Neighbour_smooth> neighbours;
    double vel_change[5][2] = {{-1*accn,accn},{0,accn},{0,0},{accn,0},{accn,-1*accn}};

    for(int counter = 0; counter<5;counter++){
        Neighbour_smooth n;
        double dV_l = vel_change[counter][0];
        double dV_r = vel_change[counter][1];
        double Vl = U_l + dV_l;
        double Vr = U_r + dV_r;
        if(Vl>1) Vl=1.0;
        if(Vl<0.4) Vl=0.4;
        if(Vr>1) Vr=1.0;
        if(Vr<0.4) Vr=0.4;
        double V = (Vl+Vr)/2;
        double w = (Vl-Vr)/d_robot;
        double theta = w*time_frame + theta_i;
        double Xf = V*cos(theta)*time_frame + Xi;
        // cout<<"vl  "<<Vl<<"vr "<<Vr;
        // cout<<"v "<<V;
        // cout<<"w  "<<w;
        // cout<<"theta"<<theta;
        // cout<<"  costheta  "<<cos(theta);
        // cout<<"  xf  "<<Xf<<endl;
        // cout<<" time frame "<<time_frame;
        double Yf = V*sin(theta)*time_frame + Yi;
        //cout<<"  yf "<<Yf<<"pos "<<Yf*width + Xf<<endl;
        if(!((Xf>=0)&&(Xf<width)&&(Yf>=0)&&(Yf<height))) continue;
        n.pos = width*Yf + Xf;
        n.orientation = correct_deg(to_deg(theta));
        n.d = V*time_frame;
        n.c = abs(dV_l) + abs(dV_r);
        n.V_l = Vl;
        n.V_r = Vr;
        neighbours.push_back(n);
    }
    return neighbours;
}


///// Astar methods:

vector<int> Astar::search_path(const int& src, const int& dest){
    //auto start_clock1 = std::chrono::steady_clock::now();

    vector<int> path;
    //ROS_INFO("checkpoint astar 1");
    if (!isValid(src)){
        cout<<"source is invalid"<<endl;
        return path;
    }
    if (!isValid(dest)){
        cout<<"dest is invalid"<<endl;
        return path;
    }
    if (!isNotBlocked(dest)){
        cout<<"dest blocked"<<endl;
        return path;
    }
    if (src==dest){
        cout<<"source and dest sameee"<<endl;
        return path;
    }

    //bool visited[maplen]; //already visited/explored cells
    //memset(visited,false,sizeof(visited));
    vector<bool> visited(maplen,false);
    //ROS_INFO("checkpoint astar 2");
    
    //array<Node,maplen> matrix;
    //Node matrix[maplen];
    vector<Node> matrix(maplen);
    int i = src;
    matrix[i].f = 0.0;
    matrix[i].g = 0.0;
    //matrix[i].h = 0.0;
    matrix[i].parent = i;
    //ROS_INFO("checkpoint astar 3");
    //passing (f,x,y) in priority queue. min_f at top
    //toExplore is to be explored nodes/cells
    std::priority_queue<Pair, std::vector<Pair>, std::greater<Pair>> toExplore;
    toExplore.emplace(0.0,i);
    //ROS_INFO("checkpoint astar 4");

    int early_successor = -1;
    while (!toExplore.empty())
    {   //auto end_clockpq = std::chrono::steady_clock::now();
        if(early_successor > -1) i = early_successor;
        else{
            const Pair& p = toExplore.top();
            i = get<1>(p);
            toExplore.pop();
        }
	    //auto start_clockpq = std::chrono::steady_clock::now();
	    //auto diff_timepq = start_clockpq - end_clockpq;
        //cout << std::chrono::duration <double, milli> (diff_timepq).count() << " ms for pq" << endl;

        if (visited[i] == true) continue; //to remove duplicate longer nodes
        visited[i] = true;
        //ROS_INFO("checkpoint astar while 1");
	    if(matrix[i].g>30000){ // won't generate full path but small goal faster
	        path = findpath(matrix,i);
	        ROS_INFO("not finding full path. uncomment");
	    return path;
	    }
	//ROS_INFO("checkpoint astar while 2");
        early_successor = -1;
        double lowest_neighbour = get<0>(toExplore.top());

        vector<int> neighbours = findNeighbours(i);
        for(auto loopvar=neighbours.rbegin();loopvar!=neighbours.rend();++loopvar){
            int neighbour = *loopvar;
            if(isNotBlocked(neighbour) && !visited[neighbour])
            {   //ROS_INFO("checkpoint astar while 3");
                if(neighbour==dest)
                {	//ROS_INFO("calling findpath");
                    matrix[neighbour].parent = i;
		    matrix[neighbour].g = matrix[i].g + heuristic(i,neighbour);
		    //cout<<"neighbours g = "<< matrix[i].g<<endl;
                    // cout<<"dest found"<<endl;
                    path.clear();

		            //auto end_clock1 = std::chrono::steady_clock::now();
		            //auto diff_time = end_clock1 - start_clock1;
        	        //cout << std::chrono::duration <double, milli> (diff_time).count() << " ms for astar" << endl;

                    path = findpath(matrix,dest);

		            //start_clock1 = std::chrono::steady_clock::now();
		            //diff_time = start_clock1 - end_clock1;
        	        //cout << std::chrono::duration <double, milli> (diff_time).count() << " ms for findpath" << endl;

                    return path; 
                }

                else
                {	//ROS_INFO("adding neighbours to be explored");
                    double g_new,h_new,f_new;
                    g_new = matrix[i].g + heuristic(i,neighbour); //1 for diagonal?
                    h_new = heuristic(neighbour,dest);
                    f_new = g_new + (g_new<h_new? 1.0000001 : 1.001)*h_new + avg_kernel_cost(neighbour,1);

                    //if(matrix[neighbour].f == -1 || matrix[neighbour].f > f_new)
		            if(matrix[neighbour].f > f_new) //when default is infinity (check .h file)
                    {   
			            matrix[neighbour].g = g_new;
                        //matrix[neighbour].h = h_new;  //removed from Node definition
                        matrix[neighbour].f = f_new;
                        matrix[neighbour].parent = i;

			            if(lowest_neighbour > f_new){
                            if(lowest_neighbour = get<0>(toExplore.top())){
                                lowest_neighbour = matrix[neighbour].f;
                                early_successor = neighbour;
				                //ROS_INFO("an early successor");
                                continue;
                            }
                            else{
				                //ROS_INFO("switching early successor");
                                lowest_neighbour = matrix[neighbour].f;
                                int temp = early_successor;
                                early_successor = neighbour;
                                neighbour = temp;
                            }
                        }

                        toExplore.emplace(matrix[neighbour].f,neighbour);
                    }
                }
                    
            }
        }
        
    }
    cout<<"dest not found"<<endl;
    

}


///// RAstar methods:

vector<int> RAstar::search_path(const int& src, const int& dest){
    auto start_clock1 = std::chrono::steady_clock::now();

    vector<int> path;
    //ROS_INFO("checkpoint astar 1");
    if (!isValid(src)){
        cout<<"source is invalid"<<endl;
        return path;
    }
    if (!isValid(dest)){
        cout<<"dest is invalid"<<endl;
        return path;
    }
    if (!isNotBlocked(dest)){
        cout<<"dest blocked"<<endl;
        return path;
    }
    if (src==dest){
        cout<<"source and dest sameee"<<endl;
        return path;
    }

    //double g_score[maplen];
    //for(int loopvar=0;loopvar<maplen;loopvar++) g_score[loopvar] = DBL_MAX;
    vector<double> g_score(maplen,DBL_MAX);
    g_score[src] = 0;
    int i = src;
    double f_i = 0.0;

    std::priority_queue<Pair, std::vector<Pair>, std::greater<Pair>> toExplore;
    toExplore.emplace(0.0,i);

    int early_successor = -1;
    while(!toExplore.empty()){
        if (early_successor !=-1) i = early_successor;
        else{
            const Pair& p = toExplore.top();
            i = get<1>(p);
            toExplore.pop();
        }
	
	    if(g_score[i]>30000){
	        path = tracePath(src,i,g_score);
	        //ROS_INFO("not finding full path. uncomment);
	        return path;
	    }		
	
        early_successor = -1;
        double lowest_neighbour = get<0>(toExplore.top());

        vector<int> neighbours = findNeighbours(i);
        for(auto loopvar=neighbours.rbegin();loopvar!=neighbours.rend();++loopvar){
            int neighbour = *loopvar;
            if(g_score[neighbour]== DBL_MAX){
		        
                g_score[neighbour] = g_score[i] + heuristic(i,neighbour);
                double h_new = heuristic(dest,neighbour);
                f_i = g_score[neighbour] + (g_score[neighbour]<h_new? 1.0000001 : 1.0001)*h_new + avg_kernel_cost(neighbour,1);

                if(lowest_neighbour > f_i){
                    if(lowest_neighbour == get<0>(toExplore.top())){
                        lowest_neighbour = f_i;
                        early_successor = neighbour;
                    }
                    else{
                        double tempp = f_i;
                        f_i = lowest_neighbour;
                        lowest_neighbour = tempp;
                        tempp = early_successor;
                        early_successor = neighbour;
                        neighbour = (int)tempp;
                    }
                }
                toExplore.emplace(f_i,neighbour);
            }

	    if(g_score[dest]!=DBL_MAX)
	    {	
		auto end_clock1 = std::chrono::steady_clock::now();
    		auto diff_time = end_clock1 - start_clock1;
    		cout << std::chrono::duration <double, milli> (diff_time).count() << " ms for rastar" << endl;
		
		path=tracePath(src,dest, g_score);

		start_clock1 = std::chrono::steady_clock::now();
	        diff_time = start_clock1 - end_clock1;
    	   	cout << std::chrono::duration <double, milli> (diff_time).count() << " ms for trackpath" << endl;

		return path; 
	    }
        }
    }
    
    
	if(g_score[dest]==DBL_MAX)
	{
		cout << "Failure to find a path !" << endl;
		return path;
	}

    
}

vector<int> RAstar::tracePath(const int& src,const int& dest,const vector<double>& g_score){
    vector<int> path;
    path.push_back(dest);
    int current = dest;

    while(current!=src){
        vector<int> neighbours = findNeighbours(current);
        std::priority_queue<Pair, std::vector<Pair>, std::greater<Pair>> tempp;
        clearQueue(tempp);
        for(auto i=neighbours.rbegin();i!=neighbours.rend();++i)
            tempp.emplace(g_score[*i],*i);
        
        Pair p = tempp.top();
        current = get<1>(p);
        path.push_back(current);
    }

    // while(current!=src){
    //     vector<int> neighbours = findNeighbours(current);
    //     vector<double> gScores;
    //     for(uint countervar=0; countervar<neighbours.size(); countervar++)
    //         gScores.push_back(g_score[neighbours[countervar]]);
    //     int minPos = distance(gScores.begin(),min_element(gScores.begin(),gScores.end()));
    //     current = neighbours[minPos];
    //     path.push_back(current);
    // }

    return path;

}

///// Tstar:

vector<int> Tstar::search_path(const int& src, const int& dest){
    //auto start_clock1 = std::chrono::steady_clock::now();

    vector<int> path;
    ROS_INFO("checkpoint astar 1");
    if (!isValid(src)){
        cout<<"source is invalid"<<endl;
        return path;
    }
    if (!isValid(dest)){
        cout<<"dest is invalid"<<endl;
        return path;
    }
    if (!isNotBlocked(dest)){
        cout<<"dest blocked"<<endl;
        return path;
    }
    if (src==dest){
        cout<<"source and dest sameee"<<endl;
        return path;
    }

    //bool visited[maplen]; //already visited/explored cells
    //memset(visited,false,sizeof(visited));
    vector<bool> visited(maplen,false);
    ROS_INFO("checkpoint astar 2");
    
    //array<Node,maplen> matrix;
    //Node matrix[maplen];
    vector<Node> matrix(maplen);
    int i = src;
    matrix[i].f = 0.0;
    matrix[i].g = 0.0;
    //matrix[i].h = 0.0;
    matrix[i].parent = i;
    ROS_INFO("checkpoint astar 3");
    //passing (f,x,y) in priority queue. min_f at top
    //toExplore is to be explored nodes/cells
    std::priority_queue<Pair, std::vector<Pair>, std::greater<Pair>> toExplore;
    toExplore.emplace(0.0,i);
    ROS_INFO("checkpoint astar 4");

    int early_successor = -1;
    while (!toExplore.empty())
    {   //auto end_clockpq = std::chrono::steady_clock::now();
        if(early_successor != -1) i = early_successor;
        else{
            const Pair& p = toExplore.top();
            i = get<1>(p);
            toExplore.pop();
        }
	    //auto start_clockpq = std::chrono::steady_clock::now();
	    //auto diff_timepq = start_clockpq - end_clockpq;
        //cout << std::chrono::duration <double, milli> (diff_timepq).count() << " ms for pq" << endl;

        if (visited[i] == true) continue; //to remove duplicate longer nodes
        visited[i] = true;
        ROS_INFO("checkpoint astar while 1");
	    if(matrix[i].g>30000){ // won't generate full path but small goal faster
	        path = findpath(matrix,i);
	        ROS_INFO("not finding full path. uncomment");
	    return path;
	    }
	ROS_INFO("checkpoint astar while 2");
        early_successor = -1;
        double lowest_neighbour = get<0>(toExplore.top());

        vector<int> neighbours = findNeighbours(i);
        for(auto loopvar=neighbours.rbegin();loopvar!=neighbours.rend();++loopvar){
            int neighbour = *loopvar;
            if(isNotBlocked(neighbour) && !visited[neighbour])
            {   ROS_INFO("checkpoint astar while 3");
                if(neighbour==dest)
                {	ROS_INFO("calling findpath");
                    matrix[neighbour].parent = i;
		    matrix[neighbour].g = matrix[i].g + heuristic(i,neighbour);
		    //cout<<"neighbours g = "<< matrix[i].g<<endl;
                    // cout<<"dest found"<<endl;
                    path.clear();

		            //auto end_clock1 = std::chrono::steady_clock::now();
		            //auto diff_time = end_clock1 - start_clock1;
        	        //cout << std::chrono::duration <double, milli> (diff_time).count() << " ms for astar" << endl;

                    path = findpath(matrix,dest);

		            //start_clock1 = std::chrono::steady_clock::now();
		            //diff_time = start_clock1 - end_clock1;
        	        //cout << std::chrono::duration <double, milli> (diff_time).count() << " ms for findpath" << endl;

                    return path; 
                }

                else
                {   //cout<<"finding neighboursszzz"<<endl;	
		            double g_new,h_new,f_new,cost_new;
                    int par;
                    cost_new = 2*line_of_sight(matrix[i].parent,neighbour);
                    if(cost_new > 0){
                        g_new = matrix[matrix[i].parent].g + heuristic(matrix[i].parent,neighbour);
                        par = matrix[i].parent;
			            //cost_new += 20*avg_kernel_cost(neighbour,5);
                    }
		            //cout<<"lineofsight ok"<<endl;
                    else{
                        g_new = matrix[i].g + heuristic(i,neighbour);
                        par = i;
                        //cost_new = 20*avg_kernel_cost(neighbour,5);
                    }
 		            cost_new = 3*avg_kernel_cost(neighbour,3);
                    h_new = heuristic(neighbour,dest);
                    f_new = g_new + (g_new<h_new? 1.0000001 : 1.001)*h_new + cost_new;

                    //if(matrix[neighbour].f == -1 || matrix[neighbour].f > f_new)
		            if(matrix[neighbour].f > f_new) //when default is infinity (check .h file)
                    {   
			            matrix[neighbour].g = g_new;
                        //matrix[neighbour].h = h_new;
                        matrix[neighbour].f = f_new;
                        matrix[neighbour].parent = par;

			            if(lowest_neighbour > f_new){
                            if(lowest_neighbour = get<0>(toExplore.top())){
                                lowest_neighbour = matrix[neighbour].f;
                                early_successor = neighbour;
				//ROS_INFO("an early successor");
                                continue;
                            }
                            else{
				//ROS_INFO("switching early successor");
                                lowest_neighbour = matrix[neighbour].f;
                                int temp = early_successor;
                                early_successor = neighbour;
                                neighbour = temp;
                            }
                        }

                        toExplore.emplace(matrix[neighbour].f,neighbour);
                    }
                }
                    
            }
        }
        
    }
    cout<<"dest not found"<<endl;
    

}



double Tstar::line_of_sight(const int& s, const int& s1){
    int x0 = s%width;
    int x1 = s1%width;
    int y0 = s/width;
    int y1 = s1/width;

    double cost = 0;
    int dy = y1-y0 , dx = x1-x0, f,sy,sx;
    if(dy<0){
        dy = -dy;
        sy = -1;
    }
    else sy = 1;

    if(dx<0){
        dx = -dx;
        sx = -1;
    }
    else sx = 1;

    if(dx>=dy){
        while(x0!=x1){
            f += dy;
            if(f>=dx && isInside(y0 + ((sy-1)/2),x0+((sx-1)/2))){
                if(costmap_->getCost(x0+((sx-1)/2), y0 + ((sy-1)/2)) >= 253)
                    return 0;
                //cost += costmap_->getCost(x0+((sx-1)/2), y0 + ((sy-1)/2));
                y0 += sy;
                f = f-dx;
            }

            if(f!=0 && isInside(y0 + ((sy-1)/2),x0+((sx-1)/2)) && (costmap_->getCost(x0+((sx-1)/2), y0 + ((sy-1)/2)) >= 253))
                return 0;
            //cost += costmap_->getCost(x0+((sx-1)/2), y0 + ((sy-1)/2));

            if(isInside(y0,x0 + ((sx-1)/2)) && isInside(y0-1,x0 + ((sx-1),2)))   
                if(dy==0 && (costmap_->getCost(x0 + ((sx-1)/2), y0) >= 253) && (costmap_->getCost(x0 + ((sx-1),2), y0-1) >= 253))
                    return 0;
            //cost = cost + costmap_->getCost(x0 + ((sx-1)/2), y0) + costmap_->getCost(x0 + ((sx-1),2), y0-1);
            x0 = x0 + sx;

        }
    }
    else{
        while(y0!=y1){
            f += dx;
            if(f>=dy  && isInside(y0 + ((sy-1)/2),x0+((sx-1)/2))){
                if(costmap_->getCost(x0+((sx-1)/2), y0 + ((sy-1)/2)) >= 253)
                    return 0;
                //cost += costmap_->getCost(x0+((sx-1)/2), y0 + ((sy-1)/2));
                x0 += sx;
                f = f - dy;
            }
            if(f!=0 && isInside(y0 + ((sy-1)/2),x0+((sx-1)/2)) && (costmap_->getCost(x0+((sx-1)/2), y0 + ((sy-1)/2)) >= 253))
                return 0;
            //cost += costmap_->getCost(x0+((sx-1)/2), y0 + ((sy-1)/2));
            if(isInside(y0+((sy-1)/2),x0) && isInside(y0 + ((sy-1)/2),x0-1))
                if(dx==0 && (costmap_->getCost(x0, y0+((sy-1)/2)) >= 253) && (costmap_->getCost(x0-1 , y0 + ((sy-1)/2)) >= 253))
                    return 0;
            //cost = cost + costmap_->getCost(x0, y0+((sy-1)/2)) + costmap_->getCost(x0-1 , y0 + ((sy-1)/2));
            y0 += sy;
        }
    }
    return cost+1;
}

};

