#include "cloop.hpp"

//-------------------------------------------
// Init Cloop
void CLOOP::CLOOP_init()
{ 
  Gmap.idx_ref_pose_slam = 0;
  Gmap.AnchorsDATA.clear();
  Gmap.KeyFDATA.clear();
  Gmap.Vgraph.clear();

  cam_parameters.distortions = &PAR.Mono_cam_distortions[0];
  cam_parameters.cc[0] = PAR.Mono_cam_cc_u;
  cam_parameters.cc[1] = PAR.Mono_cam_cc_v;
  cam_parameters.fc[0] = PAR.Mono_cam_fc_u;
  cam_parameters.fc[1] = PAR.Mono_cam_fc_v;
  cam_parameters.alpha_c = PAR.Mono_cam_alpha_c;

  Close_loop = false;

  travel_dis = 0;
  n_cloop_intents = 0;
  last_pos = {0,0,0};

  n_kf_home = 0;
  activate_CL_flag = false;
           
  // initialice store structure for log system statistics    
  store.time_search_per_step.first.clear();
  store.time_search_per_step.second.clear();
  store.cloop_stats.clear();

  cout << "cloop -> initialized" << endl;  

}

//------------------------------------------
// Set global map
// Rodrigo M. 20222
void CLOOP::Set_GlobalMap(GLOBAL_MAP &Gmap_c)
{

  Gmap = Gmap_c;

}
//------------------------------------------
// Set global map
// Rodrigo M. 20222
void CLOOP::Get_GlobalMap(GLOBAL_MAP &Gmap_c)
{
  Gmap_c = Gmap;
}

//------------------------------------------
arma::vec::fixed<3>  CLOOP::Get_delta_pos()
{
  return Delta_kf_n;      
}
//----------------------------------------
bool CLOOP::Get_close_loop_state()
{
  return Close_loop;
}
void CLOOP::Set_close_loop_state(bool state)
{
  Close_loop = state;
}
void CLOOP::Set_n_kf_rechome()
{
  n_kf_home = Gmap.KeyFDATA.size();
  cout << "Cloop-> Set number of KF belonging to home area to: "<<n_kf_home<< endl;
}
void CLOOP::Set_Activate_Deactivate_CL(bool value)
{
  activate_CL_flag = value;
}


//------------------------------------------
// Perform a full step for trying to close a loop
// Rodrigo M. 20222
void CLOOP::Step(KEYFRAME &kf_cl)
{
    static double total_comp_time = 0;
    static double step = 0;
    step++;
    
    static int n_kf_wo_cl = 0;
    // Get matches from old kf
    std::vector<int> idx_pt_matches;
    std::vector<cv::Point2d> image_points;
    int idx_kf_matched;
    
      auto ct_i = std::chrono::high_resolution_clock::now();  // take computation time

    Get_matches(kf_cl,idx_pt_matches,image_points,idx_kf_matched);
    n_kf_wo_cl++;

      auto ct_f = std::chrono::high_resolution_clock::now();
      auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(ct_f - ct_i);
      double comp_search_time_per_step = elapsed.count() * 1e-9;
      total_comp_time = total_comp_time + comp_search_time_per_step;

    double last_dis = arma::as_scalar(arma::norm(kf_cl.t_c2n - last_pos));
    if(last_dis > 2)
    {
      travel_dis += last_dis;
      last_pos = kf_cl.t_c2n;
    }          
     
    //cout << idx_pt_matches.size() << endl; 
    if (idx_pt_matches.size() > PAR.CL_min_n_matches && travel_dis > 10)
    { 
      /*
      cout << Gmap.Vgraph << endl;
      cout << "Points matched: " << endl;
      for(int k = 0; k < idx_pt_matches.size(); k++ ) cout << "Point: " << idx_pt_matches[k] << " init in kf: " << Gmap.AnchorsDATA[idx_pt_matches[k]].init_KF << endl;
      */
      bool get_pos;  
      arma::vec::fixed<3> kf_cl_pos;
      
      get_pos = Get_pos_of_current_kf(kf_cl_pos,kf_cl, idx_pt_matches, image_points,step);

      if(get_pos == true && activate_CL_flag == true)
      {
        Update_gmap(kf_cl_pos,kf_cl,idx_kf_matched);
        cout << "Distance traveled since origin/last pos update: " << travel_dis << endl;
        
        if(PAR.Stats){
          store.cloop_stats.back().distance_traveled = travel_dis;
        }          
        
        travel_dis = 0;
        n_cloop_intents = 0;
        n_kf_wo_cl = 0;
        Close_loop = true;
      }

    }

    // store statitics 
     if(PAR.Stats){
          //cout << "Cloop step:" << step << endl;
          store.total_comp_time = total_comp_time;
          store.time_search_per_step.first.push_back(step);
          store.time_search_per_step.second.push_back(comp_search_time_per_step);          
     }           

}


//------------------------------------------
// Perform a full step for trying to close a loop
// Rodrigo M. 20222
void CLOOP::Step2(KEYFRAME &kf_cl)
{
    static double total_comp_time = 0;
    static double step = 0;
    step++;
    
    static int n_kf_wo_cl = 0;
    // Get matches from old kf
    std::vector<int> idx_pt_matches;
    std::vector<cv::Point2d> image_points;
    int idx_kf_matched;
    
      auto ct_i = std::chrono::high_resolution_clock::now();  // take computation time

    Get_matches(kf_cl,idx_pt_matches,image_points,idx_kf_matched);
    n_kf_wo_cl++;

      auto ct_f = std::chrono::high_resolution_clock::now();
      auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(ct_f - ct_i);
      double comp_search_time_per_step = elapsed.count() * 1e-9;
      total_comp_time = total_comp_time + comp_search_time_per_step;

    double last_dis = arma::as_scalar(arma::norm(kf_cl.t_c2n - last_pos));
    if(last_dis > 2)
    {
      travel_dis += last_dis;
      last_pos = kf_cl.t_c2n;
    }          
     
    //cout << idx_pt_matches.size() << endl; 
    //if (idx_pt_matches.size() > PAR.CL_min_n_matches && travel_dis > 10)
    if (travel_dis > 10)
    { 
      /*
      cout << Gmap.Vgraph << endl;
      cout << "Points matched: " << endl;
      for(int k = 0; k < idx_pt_matches.size(); k++ ) cout << "Point: " << idx_pt_matches[k] << " init in kf: " << Gmap.AnchorsDATA[idx_pt_matches[k]].init_KF << endl;
      */
      bool get_pos;  
      arma::vec::fixed<3> kf_cl_pos;
      
      //get_pos = Get_pos_of_current_kf(kf_cl_pos,kf_cl, idx_pt_matches, image_points,step);
      get_pos = Get_pos_of_current_kf_from_Home_tag(kf_cl_pos,kf_cl);


      if(get_pos == true && activate_CL_flag == true)
      {
        Update_gmap(kf_cl_pos,kf_cl,idx_kf_matched);
        cout << "Distance traveled since origin/last pos update: " << travel_dis << endl;
        
        if(PAR.Stats && !store.cloop_stats.empty()){
          store.cloop_stats.back().distance_traveled = travel_dis;
        }          
        
        travel_dis = 0;
        n_cloop_intents = 0;
        n_kf_wo_cl = 0;
        Close_loop = true;
      }

    }

    // store statitics 
     if(PAR.Stats){
          //cout << "Cloop step:" << step << endl;
          store.total_comp_time = total_comp_time;
          store.time_search_per_step.first.push_back(step);
          store.time_search_per_step.second.push_back(comp_search_time_per_step);          
     }           

}