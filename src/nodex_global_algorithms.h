template <class Q>
void clearQueue(Q& q){
    q = Q();
}

namespace globalplanner{

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
    return isValid(p) && (253>kernel_cost(p,1));
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
                cost += costmap_[j + width*i];
        }
    }
    return cost;
}

double PlannerAlgorithm::avg_kernel_cost(int index, int ksize){
    double cost = kernel_cost(index,ksize);
    return cost/(ksize*ksize);
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
                    f_new = g_new + (g_new<h_new? 1.0000001 : 1.001)*h_new + avg_kernel_cost(neighbour,3);

                    //if(matrix[neighbour].f == -1 || matrix[neighbour].f > f_new)
		            if(matrix[neighbour].f > f_new) //when default is infinity (check .h file)
                    {   
			            matrix[neighbour].g = g_new;
                        //matrix[neighbour].h = h_new;
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
                f_i = g_score[neighbour] + (g_score[neighbour]<h_new? 1.0000001 : 1.0001)*h_new + avg_kernel_cost(neighbour,3);

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
    return path;

}

///// Tstar:

vector<int> Tstar::search_path(const int& src, const int& dest){
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
                {	double g_new,h_new,f_new,cost_new;
                    int par;
                    cost_new = 2*line_of_sight(matrix[i].parent,neighbour);
                    if(cost_new > 0){
                        g_new = matrix[matrix[i].parent].g + heuristic(matrix[i].parent,neighbour);
                        par = matrix[i].parent;
			//cost_new += 20*avg_kernel_cost(neighbour,5);
                    }
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
                if(costmap_[(x0+((sx-1)/2)) + width*(y0 + ((sy-1)/2))] >= 253)
                    return 0;
                //cost += costmap_->getCost(x0+((sx-1)/2), y0 + ((sy-1)/2));
                y0 += sy;
                f = f-dx;
            }

            if(f!=0 && isInside(y0 + ((sy-1)/2),x0+((sx-1)/2)) && (costmap_[(x0+((sx-1)/2)) + width*(y0 + ((sy-1)/2))] >= 253))
                return 0;
            //cost += costmap_->getCost(x0+((sx-1)/2), y0 + ((sy-1)/2));
            if(isInside(y0,x0 + ((sx-1)/2)) && isInside(y0-1,x0 + ((sx-1),2)))
                if(dy==0 && (costmap_[(x0 + ((sx-1)/2)) + width*y0] >= 253) && (costmap_[(x0 + ((sx-1),2)) + width*(y0-1)] >= 253))
                    return 0;
            //cost = cost + costmap_->getCost(x0 + ((sx-1)/2), y0) + costmap_->getCost(x0 + ((sx-1),2), y0-1);
            x0 = x0 + sx;

        }
    }
    else{
        while(y0!=y1){
            f += dx;
            if(f>=dy  && isInside(y0 + ((sy-1)/2),x0+((sx-1)/2))){
                if(costmap_[(x0+((sx-1)/2)) + width*(y0 + ((sy-1)/2))] >= 253)
                    return 0;
                //cost += costmap_->getCost(x0+((sx-1)/2), y0 + ((sy-1)/2));
                x0 += sx;
                f = f - dy;
            }
            if(f!=0 && isInside(y0 + ((sy-1)/2),x0+((sx-1)/2)) && (costmap_[(x0+((sx-1)/2)) + width*(y0 + ((sy-1)/2))] >= 253))
                return 0;
            //cost += costmap_->getCost(x0+((sx-1)/2), y0 + ((sy-1)/2));
            if(isInside(y0+((sy-1)/2),x0) && isInside(y0 + ((sy-1)/2),x0-1))
                if(dx==0 && (costmap_[x0 + width*(y0+((sy-1)/2))] >= 253) && (costmap_[(x0-1) + width*(y0 + ((sy-1)/2))] >= 253))
                    return 0;
            //cost = cost + costmap_->getCost(x0, y0+((sy-1)/2)) + costmap_->getCost(x0-1 , y0 + ((sy-1)/2));
            y0 += sy;
        }
    }
    return cost+1;
}

};

