#include "costmap.hh"

Costmap::Costmap(ignition::math::Box boundary, double resolution){
    this->boundary = boundary;
    this->resolution = resolution;

    this->top_left = ignition::math::Vector3d(boundary.Min().X(), boundary.Max().Y(), 0);
    this->width = boundary.Max().X() - boundary.Min().X();
    this->height = boundary.Max().Y() - boundary.Min().Y();

    this->cols = this->width/this->resolution;
    this->rows = this->height/this->resolution;
    
    for (int r = 0; r <this->rows; ++r){
        std::vector<int> new_row;
        std::vector<double> new_row2;
        for (int c = 0; c< this->cols; ++c){
            new_row.push_back(1);
            new_row2.push_back(10e9);
        }
        this->costmap.push_back(new_row);
        this->integration_field.push_back(new_row2);
    }

    this->last_path = this->costmap;
    this->obj_count = 0;

}

void Costmap::AddObject(ignition::math::Box object){
    object.Min().Z() = 0;
    object.Max().Z() = 0;

    auto tl = ignition::math::Vector3d(object.Min().X(), object.Max().Y(), 0);
    auto br = ignition::math::Vector3d(object.Max().X(), object.Min().Y(), 0);

    if (tl.X() >= this->boundary.Max().X() || tl.Y() <= this->boundary.Min().Y() || br.X() <= this->boundary.Min().X() || br.Y() >= this->boundary.Max().Y()){
        return;
    }

    int min_r, min_c;
    int max_r, max_c;

    this->PosToIndicies(tl, min_r, min_c);
    
    this->PosToIndicies(br, max_r, max_c);

    //std::printf("tl: (%f, %f), br: (%f, %f), min: (%d, %d), max (%d, %d)\n", tl.X(), tl.Y(), br.X(), br.Y(), min_r, min_c, max_r, max_c);

    for (int r = min_r; r<=max_r; ++r){
        for(int c = min_c; c<=max_c; ++c){
            this->costmap[r][c] = 255;
        }
    }

    this->last_path = this->costmap;
    this->obj_count++;

}

std::string Costmap::ToString(){
    std::stringstream out;

    for (int r = 0; r<this->rows; r++){
        for (int c= 0; c<this->cols; c++){
            if (this->costmap[r][c] == 1){
                out << "*";
            } else{
                out << "▇";
            }
        }
        out << "\n";
    }


    return out.str();
}

/*
* SaveFlowField writes the angles and offsets of the costmap flow field to a file.
* The flowfield is saved as a txt (angles, x_offset, y_offset, z_offset) in the "FlowField" folder where simulated runs are stored. 
* The name of the flowfield includes the endgoal to which the fields points to.
* NOT TESTED SO FAR - PROBABLY DOES NOT WORK
*
* @param end Goal where the flowfield converges (x, y, z)
*/
void Costmap::SaveFlowField(ignition::math::Vector3d end){

    std::string user_name = "default";
    if (const char * user = std::getenv("USER")){
        user_name = user;
    } 

    std::string path = "/home/" + user_name + "/Simulation_Data/FlowField/flowfield_" + std::to_string(std::trunc(end.X())) + "_" + std::to_string(std::trunc(end.Y())) + "_" + std::to_string(std::trunc(end.Z())) + ".txt";
    std::cout << path << std::endl;
    std::ofstream out(path);

    for (int r = 0; r<this->rows; r++){
        for (int c= 0; c<this->cols; c++){
            out << std::to_string(this->flow_field_angles[r][c]); 
            out << " ";
            out << std::to_string(this->flow_field_offsets[r][c]);
            out << "\n";
        }
    }
    out.flush();
    out.close();
}

/*
* ReadFlowField search if the flowfield to a given goal exists, if so it loads it into the current costmap, erasing the current values for flow_field_offsets and flow_field_angles
*
* NOT TESTED SO FAR - PROBABLY DOES NOT WORK
* @param end Goal where the flowfield converges (x, y, z)
*/
bool Costmap::ReadFlowField(ignition::math::Vector3d end){
    bool read = false;

    std::string user_name = "default";
    if (const char * user = std::getenv("USER")){
        user_name = user;
    } 

    std::string path = "/home/" + user_name + "/Simulation_Data/FlowField/flowfield_" + std::to_string(std::trunc(end.X())) + "_" + std::to_string(std::trunc(end.Y())) + "_" + std::to_string(std::trunc(end.Z())) + ".txt";

    std::ifstream in;
    in.open(path);
    if (in){
        read = true; 
        std::string line;
        while(getline(in, line)){
            std::cout << "line : " << line << std::endl;
        }
    }

    else{
        std::cout << "Cant read flowfield. " << std::endl;
        return read;
    }
    std::cout << path << std::endl;
}



std::string Costmap::PathString(std::vector<TrajPoint> path){

    this->last_path = this->costmap;
    std::stringstream out;

    for (auto pt: path){
        int r,c;
        this->PosToIndicies(pt.pose.Pos(), r, c);
        this->last_path[r][c] = -1;
    }

    

    for (int r = 0; r<this->rows; r++){
        for (int c= 0; c<this->cols; c++){
            if (this->last_path[r][c] == 1){
                out << " ";
            } else if (this->last_path[r][c] == -1) {
                out << "X";
            } else {
                out << "▇";
            }
        }
        out << "\n";
    }

    return out.str();
}

/*
* FindPath will search the Integration Field from the starting pose to the goal pose by looking for the lowest cost neighbour. 
*
*/
bool Costmap::FindPath(ignition::math::Vector3d start, ignition::math::Vector3d end,  std::vector<ignition::math::Vector3d> &path){
    if (!this->Integrate(end)){
        return false;
    }


    if (start.X() < this->top_left.X() || start.X() > this->boundary.Max().X() || start.Y() > this->top_left.Y() || start.Y() < this->boundary.Min().Y()){
        return false;
    }

    

    path.push_back(start);

    int end_r, end_c, start_r, start_c;

    this->PosToIndicies(start, start_r, start_c);
    this->PosToIndicies(end, end_r, end_c);

    if (this->integration_field[start_r][start_c] == 10e9){
        return false;
    }

    auto curr_pos = start;
    std::vector<int> curr_ind = {start_r, start_c};
    //this->last_path[curr_ind[0]][curr_ind[1]] = -1;

    while (curr_ind[0] != end_r || curr_ind[1] != end_c){

        // iterate over neighbours to find minimum cost path
        auto neighbours = this->GetNeighbours(curr_ind, true);

        std::vector<int> min_n;
        double min_val = 10e9;
        
        bool found = false;
        for (auto n : neighbours){
            
            double val = this->integration_field[n[0]][n[1]];
        
            if (val != 10e9){
                found = true;
            }
            
            if (val < min_val){
                min_val = val;
                min_n = n;
            }

        }


        if (!found){
            return false;
        } 
            

        ignition::math::Vector3d pos1, pos2;
        this->IndiciesToPos(pos1, curr_ind[0], curr_ind[1]);
        this->IndiciesToPos(pos2, min_n[0], min_n[1]);

        auto offset = pos2 - pos1;
        curr_pos += offset;
        path.push_back(curr_pos);
        curr_ind = min_n;


        //this->last_path[curr_ind[0]][curr_ind[1]] = -1;

    }

    // smooth path
    
    int check_ind = 0;
    int next_ind =1;
    while (next_ind < path.size()-1){
        if (this->Walkable(path[check_ind],path[next_ind+1])){
            path.erase(path.begin()+next_ind);
        } else{
            check_ind = next_ind;
            next_ind = check_ind +1;
        }
    }

    return true;
}

/*
* ComputeFlowField compute a flow field offset (difference in integration field between current grid pose and lowest neighbouring integration field) and angle at each point of the costmap to a given goal.
*  It finds the lowest cost neighbours at each point in the grid, and compute the vector from the current position to this neighbour. 
* 
* @param end goal to reach
*/
void Costmap::ComputeFlowField(ignition::math::Vector3d end){
    if (!this->Integrate(end)){
        std::cout << "ERROR INTEGRATING COSTMAP INTEGRATION FIELD." << std::endl;
        return;
    }

    int goal_r, goal_c;
    this->PosToIndicies(end, goal_r, goal_c);
    
    std::vector<int> curr_ind; 
    //int count = 0;
    for (int r = 0; r<this->rows; r++){
        std::vector<double> new_row_offsets;
        std::vector<double> new_row_angles;

        for (int c = 0; c<this->cols; c++){
            // Set offset and angle of flow field to 0 on the sides of the map 
            if(r==0 || r==this->rows-1){ 
                new_row_offsets.push_back(0);
                new_row_angles.push_back(0);
            }
            else{
                if(c==0 || c==this->cols-1){
                    new_row_offsets.push_back(0);
                    new_row_angles.push_back(0);
                }
                else{
                    curr_ind = {r, c};
                    if (this->costmap[r][c] == 255){// set to 0 if within an obstacle
                        new_row_offsets.push_back(0);
                        new_row_angles.push_back(0);
                        continue;
                    }
                    
                    auto neighbours = this->GetNeighbours(curr_ind, true); 
                    std::vector<int> min_n = curr_ind;
                    double min_val = 10e9;
                    for (auto n : neighbours){
                        double val = this->integration_field[n[0]][n[1]];

                        if (val < min_val){
                            min_val = val;
                            min_n = n;
                        }
                    }
                    // if(min_val != 10e9){
                    //     count += 1;
                    // }

                    new_row_offsets.push_back(min_val - this->integration_field[curr_ind[0]][curr_ind[1]]);
                    new_row_angles.push_back(GetNeighbourAngle(curr_ind, min_n));
                    // std::cout << "New offset: " << min_val - this->integration_field[curr_ind[0]][curr_ind[1]] << std::endl;
                    // std::cout << "Min val: " << min_val << " curr val: " << this->integration_field[curr_ind[0]][curr_ind[1]] << std::endl; 
                    // std::cout << "Angle : " << GetNeighbourAngle(curr_ind, min_n) << std::endl;
                }
            }
        }
        this->flow_field_offsets.push_back(new_row_offsets);
        this->flow_field_angles.push_back(new_row_angles);
    }
    // std::cout << "Count: " << count << std::endl;
    // std::cout << "Size of flow_field_offset: " << this->flow_field_offsets.size() << " rows " << this->flow_field_offsets[1].size() << " columns" << std::endl;
    // std::cout << "Size of flow_field_angles: " << this->flow_field_angles.size() << " rows " << this->flow_field_angles[1].size() << " columns" << std::endl;
    // std::cout << "costmap width x height: " << this->width << " x " << this->height << std::endl;
    // std::cout << "costmap resolution; " << this->resolution << std::endl; 
    // std::cout << "costmap row col: " << this->rows << " x " << this->cols << std::endl; 
}

/*
* ComputeFlowFieldFine compute a flow field offset (difference in integration field between current grid pose and lowest neighbouring integration field) and angle at each point of the costmap to a given goal.
*  This finer version computes the local gradient value in the integration map
* 
* @param end goal to reach
*/
void Costmap::ComputeFlowFieldFine(ignition::math::Vector3d end){
    static const double TWOPI = 6.2831853071795865;
    static const double RAD2DEG = 57.2957795130823209;

    if (!this->Integrate(end)){
        std::cout << "ERROR INTEGRATING COSTMAP INTEGRATION FIELD." << std::endl;
        return;
    }

    int goal_r, goal_c;
    this->PosToIndicies(end, goal_r, goal_c);

    // We get gradient with Farid and Simocelly filter
    int farid_n = 2;
    double farid_5_k[5]= {0.030320,  0.249724,  0.439911,  0.249724,  0.030320};
    double farid_5_d[5]= {0.104550,  0.292315,  0.000000, -0.292315, -0.104550};
    double farid_5_dd[5]= {-0.104550,  -0.292315,  0.000000, 0.292315, 0.104550};

    // Init convolve derivative
    std::vector<std::vector<double>> int_field_2(rows, std::vector<double>(cols, 0));
    std::vector<std::vector<double>> dx_final(rows, std::vector<double>(cols, 0));
    std::vector<std::vector<double>> dy_final(rows, std::vector<double>(cols, 0));
    std::vector<std::vector<double>> dx_tmp(rows, std::vector<double>(cols, 0));
    std::vector<std::vector<double>> dy_tmp(rows, std::vector<double>(cols, 0));

    // First get rid of the 10e9 values in range of the convolution kernel. Set them to the highest value in the kernel range
    for (int c = 0; c<this->cols; c++)
    {
        int c1 = -farid_n;
        if (c + c1 < 0)
            c1 = -c;

        int c2 = farid_n;
        if (c + c2 > cols-1)
            c2 = cols - 1 - c;

        for (int r = 0; r<this->rows; r++)
        {
            int r1 = -farid_n;
            if (r + r1 < 0)
                r1 = -r;

            int r2 = farid_n;
            if (r + r2 > rows-1)
                r2 = rows - 1 - r;

            double v0 = integration_field[r][c];
            if (v0 < 10e8)
            {
                int_field_2[r][c] = v0;
            }
            else
            {
                double max_v = 0;
                for (int cc = c1; cc <= c2; cc++)
                {
                    for (int rr = r1; rr <= r2; rr++)
                    {
                        double v = integration_field[r + rr][c + cc];
                        if (v < 10e8 && v > max_v)
                            max_v = v;
                    }
                }
                int_field_2[r][c] = max_v;
            }
        }
    }

    // Convolution along columns
    for (int c = 0; c<this->cols; c++)
    {
        int b1 = -farid_n;
        if (c + b1 < 0)
            b1 = -c;

        int b2 = farid_n;
        if (c + b2 > cols-1)
            b2 = cols - 1 - c;

        for (int r = 0; r<this->rows; r++)
        {

            double dx = 0.0;
            double dy = 0.0;
            double v0 = int_field_2[r][c];

            // First convolve each column
            for (int b = b1; b <= b2; b++)
            {
                double integration_v = int_field_2[r][c + b];
                dx += integration_v * farid_5_d[b - b1];
                dy += integration_v * farid_5_k[b - b1];
            }
            dx_tmp[r][c] = dx;
            dy_tmp[r][c] = dy;
        }
    }

    // Second convolution along rows
    for (int r = 0; r<this->rows; r++)
    {
        int b1 = -farid_n;
        if (r + b1 < 0)
            b1 = -r;

        int b2 = farid_n;
        if (r + b2 > rows-1)
            b2 = rows - 1 - r;

        for (int c = 0; c<this->cols; c++)
        {
            double dx = 0.0;
            double dy = 0.0;

            // First convolve each column
            for (int b = b1; b <= b2; b++)
            {
                dx += dx_tmp[r + b][c] * farid_5_k[b - b1];
                dy += dy_tmp[r + b][c] * farid_5_dd[b - b1];
            }
            dx_final[r][c] = dx;
            dy_final[r][c] = dy;
        }
    }

    
    for (int r = 0; r<this->rows; r++)
    {

        std::vector<double> new_row_offsets;
        std::vector<double> new_row_angles;

        for (int c = 0; c<this->cols; c++)
        {
            if (integration_field[r][c] > 10e8){// set to 0 if within an obstacle
                new_row_offsets.push_back(0);
                new_row_angles.push_back(0);
                continue;
            }

            new_row_offsets.push_back(sqrt(dx_final[r][c] * dx_final[r][c] + dy_final[r][c] * dy_final[r][c]));
            double theta = std::atan2(dy_final[r][c], dx_final[r][c]);
            if(theta<0.0)
                theta += TWOPI;
            new_row_angles.push_back(RAD2DEG * theta);
        }
        flow_field_offsets.push_back(new_row_offsets);
        flow_field_angles.push_back(new_row_angles);
    }
    // std::cout << "Count: " << count << std::endl;
    // std::cout << "Size of flow_field_offset: " << this->flow_field_offsets.size() << " rows " << this->flow_field_offsets[1].size() << " columns" << std::endl;
    // std::cout << "Size of flow_field_angles: " << this->flow_field_angles.size() << " rows " << this->flow_field_angles[1].size() << " columns" << std::endl;
    // std::cout << "costmap width x height: " << this->width << " x " << this->height << std::endl;
    // std::cout << "costmap resolution; " << this->resolution << std::endl; 
    // std::cout << "costmap row col: " << this->rows << " x " << this->cols << std::endl; 
}


bool Costmap::Walkable(ignition::math::Vector3d start, ignition::math::Vector3d end){
    // sample points every 1/5th of resolution along the line and check if it is in an occupied cell.

    auto dir = end-start;
    double length = dir.Length();
    int num = 10;
    int N = (int) length/(this->resolution/num);
    dir = dir.Normalize();
    dir*= this->resolution/num;

    for (int i =1; i <= N; i++){
        auto check_point = dir*i + start;
        int r,c;
        this->PosToIndicies(check_point, r, c);

        if (this->costmap[r][c] != 1){
            return false;
        }
    }

    return true;
}


/*
* Integrate initialized the integration field at 10e9. It sets the goal position to 0 in the field, then iteratively calculate the value of the integration field from the goal following the algorithm here
* https://leifnode.com/2013/12/flow-field-pathfinding/
*
*/
bool Costmap::Integrate(ignition::math::Vector3d goal){
    double sqrt2 = sqrt(2);

    // Init integration field to very high value
    for (int r = 0; r <this->rows; ++r){
        
        for (int c = 0; c< this->cols; ++c){
            this->integration_field[r][c] = 10e9;
        }   
        
    }
    
    // Check if goal is in boundary
    if (goal.X() < this->top_left.X() || goal.X() > this->boundary.Max().X() || goal.Y() > this->top_left.Y() || goal.Y() < this->boundary.Min().Y()){
        return false;
    }

    // Get position of the goal in the costmap
    int goal_r, goal_c;
    this->PosToIndicies(goal, goal_r, goal_c);

    // Set value for goal at 0
    this->integration_field[goal_r][goal_c] = 0;

    // Init the region growing container
    std::vector<std::vector<int>> open_list;
    open_list.push_back({goal_r, goal_c});

    // Grow region
    while (open_list.size() > 0)
    {
        // Get current candidate. This could be better implemented with pop()
        auto curr_ind = open_list.front();
        open_list.erase(open_list.begin());


        // Get list of neighbors of the candidate
        auto neighbours = this->GetNeighbours(curr_ind, true);

        // Deal with all neighbors
        for (auto n: neighbours)
        {
            // Init cost at the current value
            double n_cost = this->integration_field[curr_ind[0]][curr_ind[1]];
            
            // Add cost of getting to a costmap pixel.
            if ((n[0] - curr_ind[0]) * (n[1] -curr_ind[1]) == 0)
            {
                n_cost += (double)costmap[n[0]][n[1]];
            }
            else
            {
                n_cost += sqrt2 * (double)costmap[n[0]][n[1]];
            }

            // If we are not in an obstacle and the computed cost is lower than current cost, update it
            if (n_cost < this->integration_field[n[0]][n[1]] && this->costmap[n[0]][n[1]] < 255)
            {
                this->integration_field[n[0]][n[1]] = n_cost;
                if (std::find(open_list.begin(), open_list.end(), n) == open_list.end())
                {
                    open_list.push_back(n); 
                }
            }
        }
    }
    // START TEST BLOCK

    // int count_1_ff = 0;
    // int count_1_c = 0;
    // int count_2_ff = 0;
    // int count_2_c = 0;
    // for (int r=0; r<this->rows; r++){
    //     for (int c=0; c<this->cols; c++){
    //         if (this->integration_field[r][c] == 10e9){
    //             count_1_ff += 1;
    //         }
    //         if (this->costmap[r][c] == 255){
    //             count_1_c += 1;
    //         }
    //         if(this->integration_field[r][c] < 10e9){
    //             count_2_ff +=1;
    //         }
    //         if(this->costmap[r][c] < 255){
    //             count_2_c += 1;
    //         }
    //     }
    // }
    // std::cout << "INTEGRATE 10e9 cells: " << count_1_ff << std::endl; 
    // std::cout << "INTEGRATE < cells:  " << count_2_ff << std::endl;
    // std::cout << "Costmap values at 255: " << count_1_c << " under " << count_2_c << std::endl; 
    // END TEST BLOCK
    return true;

}

std::vector<std::vector<int>> Costmap::GetNeighbours(std::vector<int> curr_ind, bool diag){
    std::vector<std::vector<int>> res;

    if (curr_ind[0] > 0){ // we can return top
        res.push_back({curr_ind[0]-1, curr_ind[1]});
    }

    if (curr_ind[1] > 0){ // we can return left
        res.push_back({curr_ind[0], curr_ind[1]-1});
    }

    if (curr_ind[0] < this->rows-1){ // we can return bot
        res.push_back({curr_ind[0]+1, curr_ind[1]});
    }

    if (curr_ind[1] < this->cols-1){ // we can return right
        res.push_back({curr_ind[0], curr_ind[1]+1});
    }

    if (diag){
        if (curr_ind[0] > 0 && curr_ind[1] > 0){ // we can return top left
            res.push_back({curr_ind[0]-1,curr_ind[1]-1});
        }

        if (curr_ind[0] > 0 && curr_ind[1] <this->cols-1){ // we can return bottom left
            res.push_back({curr_ind[0]-1,curr_ind[1]+1});
        }

        if (curr_ind[0] < this->rows-1 && curr_ind[1] > 0){ // we can return top right
            res.push_back({curr_ind[0]+1,curr_ind[1]-1});
        }

        if (curr_ind[0] < this->rows-1 && curr_ind[1] < this->cols-1){ // we can return bottom right
            res.push_back({curr_ind[0]+1,curr_ind[1]+1});
        }
    }

    return res;
}

/*
* GetNeighbourAngle returns the angle in degres between the current grid position and a given neighbouring positions. 
* Works on the [x_min-1, x_max-1][y_min-1, y_max-1] range of the grid. Intended to use for Flow Field generation.
*
* @param curr_ind current grid indices
* @param neighbour_ind neighbour grid indices
*/
double Costmap::GetNeighbourAngle(std::vector<int> curr_ind, std::vector<int> neighbour_ind){
    static const double TWOPI = 6.2831853071795865;
    static const double RAD2DEG = 57.2957795130823209;

    if (curr_ind == neighbour_ind){
        return 0.0;
    }

    double theta = std::atan2(neighbour_ind[0] - curr_ind[0], curr_ind[1] - neighbour_ind[1]);
    if(theta<0.0){
        theta += TWOPI;
    }
    // std::cout << "Angle in deg: " << RAD2DEG * theta << " between positions: " << curr_ind[0] << " " << curr_ind[1] << " and " << neighbour_ind[0] << " " << neighbour_ind[1] << std::endl;
    return RAD2DEG * theta;
}

bool Costmap::PosToIndicies(ignition::math::Vector3d pos, int &r, int &c){
    
    r = (int)floor((top_left.Y() - pos.Y()) / resolution);
    c = (int)floor((pos.X() - top_left.X()) / resolution);

    /*
    int r0 = 0;
    int c0 = 0;
    while ((this->top_left.Y() - r*this->resolution - this->resolution) > pos.Y()){
        r++;
    }

    while ((this->top_left.X() + c*this->resolution + this->resolution) < pos.X()){
        c++;
    }
    if (r != r0 || c != c0)
    {
        std::cout << "----------------------------------- " << std::endl;
        std::cout << "---------------------> r0 = " << r0 << " but r = " << r << "  / max = " << rows << std::endl;
        std::cout << "---------------------> c0 = " << c0 << " but c = " << c << "  / max = " << cols << std::endl;
        std::cout << "----------------------------------- " << std::endl;
    }
    */

    return utilities::inside_box(this->boundary, pos, true);
}

bool Costmap::IndiciesToPos(ignition::math::Vector3d &pos, int r, int c){

    pos = ignition::math::Vector3d(this->boundary.Min().X() + c*this->resolution, this->boundary.Max().Y() - r*this->resolution, 0);
    return ((r>=0 && r < this->rows) && (c>=0 && c < this->cols));
}

double Costmap::Heuristic(std::vector<int> loc1, std::vector<int> loc2){
    ignition::math::Vector3d pos1, pos2;
    this->IndiciesToPos(pos1, loc1[0],loc1[1]);
    this->IndiciesToPos(pos2, loc2[0],loc2[1]);

    return (pos1-pos2).Length();
}

bool Costmap::Occupied(ignition::math::Vector3d pos){
    int r,c;
    this->PosToIndicies(pos, r, c);
    
    return (this->costmap[r][c] > 1);
}

ignition::math::Vector3d Costmap::RandPos(){
    // select random height
    int rand_row = ignition::math::Rand::IntUniform(0, this->rows-1);

    // work across and save safe spaces (those that are after only one collision)

    
    bool found = false;
    int count = 0;
    
    while (!found && count < 1000){
        
        std::vector<int> collisions;

        for (int c = 0; c < this->cols; c++){

            bool occupied = (this->costmap[rand_row][c] >1);

            if (occupied){

                if (c == 0){
                    if (this->costmap[rand_row][1] == 1){
                        collisions.push_back(c);
                        collisions.push_back(c);
                    }else{
                        collisions.push_back(c);
                    }
                } else if (c == this->cols-1){
                    if (this->costmap[rand_row][c-1] == 1){
                        collisions.push_back(c);
                        collisions.push_back(c);
                    }else{
                        collisions.push_back(c);
                    }
                } else{
                    if (this->costmap[rand_row][c-1] == 1 && this->costmap[rand_row][c+1] == 1){
                        collisions.push_back(c);
                        collisions.push_back(c);
                    } else if (this->costmap[rand_row][c-1] == 1 || this->costmap[rand_row][c+1] == 1){
                        collisions.push_back(c);
                    }
                }
            }


        }

        std::vector<int> safe_cols;

        for (int ind = 1; ind < collisions.size(); ind+=2){
            int start_c = collisions[ind];
            int end_c = this->cols;
            if (ind+1 < collisions.size()){
                end_c = collisions[ind+1];
            }

            for (int i = start_c+1; i < end_c; i++){
                safe_cols.push_back(i);
            }
        
        }

        if (safe_cols.size() > 0){
            found = true;
        } else {
            count ++;
            continue;
        }

        int rand_ind = ignition::math::Rand::IntUniform(0,safe_cols.size()-1);

        ignition::math::Vector3d res;
        this->IndiciesToPos(res, rand_row, safe_cols[rand_ind]);

        return res;
    }

    std::cout << "Failed to find random target for actor\n" << std::endl;
    return ignition::math::Vector3d(0,0,0);
}

bool Costmap::AStar(ignition::math::Vector3d start, ignition::math::Vector3d end, std::vector<ignition::math::Vector3d> &path, bool straighten){
    auto t1 = std::chrono::high_resolution_clock::now();

    this->parent.clear();
    this->g_cost.clear();
    this->open.clear();
    
    int start_r, start_c, end_r, end_c;
    this->PosToIndicies(start, start_r, start_c);
    this->PosToIndicies(end, end_r, end_c);


    std::vector<int> start_coords = {start_r, start_c};
    std::vector<int> end_coords = {end_r, end_c};
    //std::cout << "end: ";
    
    this->target = end_coords;

    this->g_cost[start_coords] = 0;
    this->parent[start_coords] = start_coords;
    
    this->open.put(start_coords, this->Heuristic(start_coords, end_coords));
    std::set<std::vector<int>> closed;

    bool found = false;

    while (!this->open.empty()){
        //std::cout << open.size() << std::endl;
        
        auto s = this->open.get();
        //print_coords(s);
        if (s[0] == end_coords[0] && s[1] == end_coords[1]){
            found = true;
            break; // path found
        }

        closed.insert(s);

        for (auto n: this->GetNeighbours(s, true)){
            double n_cost = this->costmap[n[0]][n[1]];
            if (n_cost > 1){ // if we encounter a wall, skip 
                continue;
            }
            if (closed.find(n) == closed.end()){
                if (this->open.find(n) == open.last()){
                    this->g_cost[n] = std::numeric_limits<double>::infinity();
                }
                this->UpdateVertexA(s, n);
            }
        }
    }


    if (!found){
        std::cout << "No Path Found\n";
        return false;
    }


    auto curr_coords = end_coords;
    ignition::math::Vector3d actual_pos = end;
    ignition::math::Vector3d last_pos = end;

    int count = 0;

    while (curr_coords[0] != start_coords[0] || curr_coords[1] != start_coords[1]){

        if (count != 0){
            ignition::math::Vector3d curr_pos;
            this->IndiciesToPos(curr_pos, curr_coords[0], curr_coords[1]);
            auto offset = curr_pos - last_pos;
            actual_pos = actual_pos+offset;
            path.push_back(actual_pos);
        } else{
            path.push_back(end);
        }

        this->IndiciesToPos(last_pos, curr_coords[0], curr_coords[1]);
        curr_coords = this->parent[curr_coords];
        

        count ++;
    }

    path.push_back(start);
    std::reverse(path.begin(), path.end());

    if (straighten){
        int check_ind = 0;
        int next_ind =1;
        while (next_ind < path.size()-1){
            if (this->Walkable(path[check_ind],path[next_ind+1])){
                path.erase(path.begin()+next_ind);
            } else{
                check_ind = next_ind;
                next_ind = check_ind +1;
            }
        }
    }

    auto t2 = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
    std::cout <<  "Path Found (A*). Duration: " << ((double)duration)/10e6 << " s"<< std::endl;
    return true;
}

void Costmap::UpdateVertexA(std::vector<int> s, std::vector<int> n){
    auto c = this->DistCost(s,n);
    if (this->g_cost[s] + c < this->g_cost[n]){
        this->g_cost[n] = this->g_cost[s] + c;
        this->parent[n] = s;
        if (this->open.find(n) != this->open.last()){
            this->open.remove(n);
        }
        this->open.put(n, this->g_cost[n] + this->Heuristic(n, this->target));
    }
}

bool Costmap::ThetaStar(ignition::math::Vector3d start, ignition::math::Vector3d end, std::vector<ignition::math::Vector3d> &path){
    auto t1 = std::chrono::high_resolution_clock::now();

    this->parent.clear();
    this->g_cost.clear();
    this->open.clear();
    
    int start_r, start_c, end_r, end_c;
    this->PosToIndicies(start, start_r, start_c);
    this->PosToIndicies(end, end_r, end_c);


    std::vector<int> start_coords = {start_r, start_c};
    std::vector<int> end_coords = {end_r, end_c};
    //std::cout << "end: ";
    
    this->target = end_coords;

    this->g_cost[start_coords] = 0;
    this->parent[start_coords] = start_coords;
    
    this->open.put(start_coords, this->Heuristic(start_coords, end_coords));
    std::set<std::vector<int>> closed;

    bool found = false;

    while (!this->open.empty()){
        //std::cout << open.size() << std::endl;
        
        auto s = this->open.get();
        //print_coords(s);
        if (s[0] == end_coords[0] && s[1] == end_coords[1]){
            found = true;
            break; // path found
        }

        closed.insert(s);

        for (auto n: this->GetNeighbours(s, true)){
            double n_cost = this->costmap[n[0]][n[1]];
            if (n_cost > 1){ // if we encounter a wall, skip 
                continue;
            }
            if (closed.find(n) == closed.end()){
                if (this->open.find(n) == open.last()){
                    this->g_cost[n] = std::numeric_limits<double>::infinity();
                }
                this->UpdateVertexB(s, n);
            }
        }
    }


    if (!found){
        return false;
    }


    auto curr_coords = end_coords;
    ignition::math::Vector3d actual_pos = end;
    ignition::math::Vector3d last_pos = end;

    int count = 0;

    while (curr_coords[0] != start_coords[0] || curr_coords[1] != start_coords[1]){

        if (count != 0){
            ignition::math::Vector3d curr_pos;
            this->IndiciesToPos(curr_pos, curr_coords[0], curr_coords[1]);
            auto offset = curr_pos - last_pos;
            actual_pos = actual_pos+offset;
            path.push_back(actual_pos);
        } else{
            path.push_back(end);
        }

        this->IndiciesToPos(last_pos, curr_coords[0], curr_coords[1]);
        curr_coords = this->parent[curr_coords];
        

        count ++;
    }

    path.push_back(start);
    std::reverse(path.begin(), path.end());

    auto t2 = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
    std::cout <<  "Path Found (Theta*). Duration: " << ((double)duration)/10e6 << " s"<< std::endl;
    return true;
}

void Costmap::UpdateVertexB(std::vector<int> s, std::vector<int> n){

    ignition::math::Vector3d p1, p2;

    auto par = this->parent[s];

    this->IndiciesToPos(p1, par[0], par[1]);
    this->IndiciesToPos(p2, n[0], n[1]);

    if (this->Walkable(p1, p2)){


        auto c = this->DistCost(par, n);
        if (this->g_cost[par] + c < this->g_cost[n]){
            this->g_cost[n] = this->g_cost[par]+c;
            this->parent[n] = par;
            if (this->open.find(n) != this->open.last()){
                this->open.remove(n);
            }
            this->open.put(n, this->g_cost[n]+this->Heuristic(n, this->target));
        }
    } else{
        auto c = this->DistCost(s, n);
        if (this->g_cost[s] + c < this->g_cost[n]){
            this->g_cost[n] = this->g_cost[s]+c;
            this->parent[n] = s;
            if (this->open.find(n) != this->open.last()){
                this->open.remove(n);
            }
            this->open.put(n, this->g_cost[n] + this->Heuristic(n, this->target));
        }
    }
}

double Costmap::DistCost(std::vector<int> s, std::vector<int> n){
    ignition::math::Vector3d s_pos;
    ignition::math::Vector3d n_pos;
    this->IndiciesToPos(s_pos, s[0], s[1]);
    this->IndiciesToPos(n_pos, n[0], n[1]);

    return (s_pos-n_pos).Length();
}