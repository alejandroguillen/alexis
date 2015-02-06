/*
 * Algorithms.cpp
 *
 *  Created on: 26/jan/2015
 *      Author: Alejandro Guillen
 */

#include "Algorithms.h"
#include "LoadBalancingConfig.h"
#include <fstream>


#define POS_Xi(i) (i+1) //i starting from 0
#define POS_T num_using_nodes_+1
#define POS_Dij(i,j) (num_using_nodes_+2)+(i)*(num_quantiles_+2)+(j)
#define POS_Fi(i) num_using_nodes_+2+(num_using_nodes_-1)*(num_quantiles_+2)+(i)
#define POS_IPi(i) (num_using_nodes_+2)+(num_using_nodes_-1)*(num_quantiles_+2)+(num_using_nodes_-1)+(i)

Algorithms::Algorithms() {
	fdr_=0;
	lastM=0;
	is_lpmodel_created_=false;
	quantiles_available_=false;
	lp_D=NULL;
	lp_G=NULL;
	lp_E=NULL;

	//Default configuration:
	reconstruction_method_=RECONSTRUCTION_METHOD_DEFAULT;
	bdr_update_coef_=BDR_UPDATE_COEF_DEFAULT;
	fdr_update_coef_=FDR_UPDATE_COEF_DEFAULT;
	scaling_coef_=SCALING_COEF_DEFAULT;

	num_quantiles_=NUM_QUANTILES_DEFAULT;
	solver_timeout_=SOLVER_TIMEOUT_DEFAULT;

	use_fixed_uniform_cuts_=USE_FIXED_UNIFORM_CUTS_DEFAULT;
	
	alpha_d_ = ALPHA_D_DEFAULT;

	//Try to read config file in "LoadBalancing_config.txt":
	std::ifstream configfile(LB_CONFIG_FILE);
	if(configfile){
		std::cout << "Algorithms: Reading config from " <<  LB_CONFIG_FILE << std::endl;
		LoadBalancingConfig lbconfig;
		lbconfig.ParseConfigFile(LB_CONFIG_FILE);
		LoadNewConfig(lbconfig);
	}
}

Algorithms::Algorithms(LoadBalancingConfig config) {
	fdr_=0;
	lastM=0;
	is_lpmodel_created_=false;
	quantiles_available_=false;
	lp_D=NULL;
	lp_G=NULL;
	lp_E=NULL;

	//Get config from 'config':
	reconstruction_method_=config.reconstruction_method;
	bdr_update_coef_=config.bdr_update_coef;
	fdr_update_coef_=config.fdr_update_coef;
	scaling_coef_=config.scaling_coef;

	num_quantiles_=config.num_quantiles;
	solver_timeout_=config.solver_timeout;

	use_fixed_uniform_cuts_=config.use_fixed_uniform_cuts;
	
	alpha_d_ = config.alpha_d;
}

void Algorithms::LoadNewConfig(LoadBalancingConfig config) {
	quantiles_available_=false;

	reconstruction_method_=config.reconstruction_method;
	bdr_update_coef_=config.bdr_update_coef;
	fdr_update_coef_=config.fdr_update_coef;
	scaling_coef_=config.scaling_coef;

	num_quantiles_=config.num_quantiles;
	solver_timeout_=config.solver_timeout;

	use_fixed_uniform_cuts_=config.use_fixed_uniform_cuts;
	alpha_d_ = config.alpha_d;
}

void Algorithms::SetImageParameters(int width, int height, double overlap) {
	height_ = height;
	width_ = width;
	overlap_ = overlap;
//TODO?	lp_delete_model();
}

void Algorithms::SetNumQuantiles(int Q) {
	num_quantiles_ = Q;
	quantiles_available_ = false;
	IPx_quantile_aprox_.clear();
}

void Algorithms::SetTargetKeypoints(int target_num_keypoints) {
	if(target_num_keypoints != Mopt_){
		Mopt_ = target_num_keypoints;
		fdr_ = 0;
		bdr_.clear();
	}
}

double Algorithms::getAlphad(){
	return alpha_d_;
}

void Algorithms::AddKeypoints(vector<KeyPoint>& kpts) {
	//	std::cout << "Algorithms::AddKeypoints = " << kpts.size() << std::endl;

	//Quantile approximation of the x-coordinates:
	//Sort IP by ascending x coordinate
	if(kpts.size() == 0){
		return;
	}
	std::vector<int> IPxcoords_sorted;
	for(int i=0;i<kpts.size();i++){
		IPxcoords_sorted.push_back(kpts[i].pt.x);
	}
	sort(IPxcoords_sorted.begin(), IPxcoords_sorted.end());

	//quantile aproximation:
	if(IPx_quantile_aprox_.size()!=num_quantiles_){
		IPx_quantile_aprox_.resize(num_quantiles_);
	}
	for(int i=0; i<num_quantiles_; i++){
		int num_IP=(int)IPxcoords_sorted.size();
		IPx_quantile_aprox_.at(i)=IPxcoords_sorted.at(std::max(num_IP*(i+1)/num_quantiles_-1, 0));
	}
	quantiles_available_=true;

	//Get keypoint responses:
	lastIPscores_.clear();
	for(int i=0;i<kpts.size();i++){
		lastIPscores_.push_back(kpts[i].response);
	}
	lastM = (int)lastIPscores_.size();
	sort(lastIPscores_.begin(), lastIPscores_.end(), std::greater<float>()); //Sort the scores in descending order
//for(size_t i=0; i<lastIPscores_.size(); i++){
//	std::cerr << lastIPscores_[i] << "; ";
//}
//std::cerr << std::endl;
//if(lastIPscores_.size() >0) std::cerr << "Lowest keypoint score (" << lastIPscores_.size() << "): " << lastIPscores_.back() << std::endl;
//if(lastIPscores_.size() >= Mopt_) std::cerr << "Keypoint (M*) response =" << lastIPscores_[Mopt_-1] << std::endl;

	if(lastM > Mopt_){
		UpdateBDR();
		UpdateFDR();
	}
}

float Algorithms::GetNextDetectionThreshold() {
	if(reconstruction_method_==RECONSTRUCTION_METHOD_FORWARD){
		//For forward estimation and last value prediction
		if(lastM >= Mopt_){
			lastThreshold_= lastIPscores_.at(Mopt_-1);
			return lastThreshold_;
		}else{
			if(fdr_ == 0){ //Training period
				lastThreshold_ = lastThreshold_/2;
				return lastThreshold_;
			}else{
				lastThreshold_= std::max(lastThreshold_-(lastM-Mopt_)*fdr_, (float)0);
				return lastThreshold_;
			}
		}
	}else if(reconstruction_method_==RECONSTRUCTION_METHOD_BACKWARD){
		//For backward estimation and last value prediction
		if(lastM >= Mopt_){
			lastThreshold_= lastIPscores_.at(Mopt_-1);
			return lastThreshold_;
		}else{
			if(bdr_.size() == 0){ //Training period
				lastThreshold_ = lastThreshold_/2;
				return lastThreshold_;
			}else{
				lastThreshold_= std::max(lastThreshold_-(lastM-Mopt_)*bdr_.at(Mopt_-lastM), (float)0);
				return lastThreshold_;
			}
		}
	}else if(reconstruction_method_==RECONSTRUCTION_METHOD_SCALING){
		if(lastM >= Mopt_){
			lastThreshold_= lastIPscores_.at(Mopt_-1);
			return lastThreshold_;
		}else{
			if(lastM == 0){
				lastThreshold_ = lastThreshold_/2;
				return lastThreshold_;
			}
			lastThreshold_= lastThreshold_*scaling_coef_;
			return lastThreshold_;
		}
	}
}

void Algorithms::CutVectorOptimization(int num_cooperators,
		vector<double>& c, vector<double>& p) {
	int ret;
	num_using_nodes_=num_cooperators;
	if(num_cooperators == 1){
		optimal_cutvector_.resize(1);
		optimal_cutvector_[0] = width_;
		//T = C*h*overlap + P(hw + alphad*Nip)
		est_completion_time_=c[0]*height_*width_ + p[0]*(height_*width_ + alpha_d_*Mopt_);
	}
	else{
		if(use_fixed_uniform_cuts_ == 1){
			optimal_cutvector_.resize(num_cooperators);
			for(int i=0; i<num_cooperators; i++){
				optimal_cutvector_[i] = round(width_*(i+1)/num_cooperators);
			}
			return;
		}
		lp_matrix_formulation(c, p);
		lp_create_model();
		ret = lp_solve_model();
		lp_delete_model();

		if(ret != 0){ //Solver error
			//If we don't have a previous solution, set uniform cuts
			if(optimal_cutvector_.size() != num_cooperators){
				optimal_cutvector_.resize(num_cooperators);
				for(int i=0; i<num_cooperators; i++){
					optimal_cutvector_[i] = round(width_*(i+1)/num_cooperators);
				}
			}
		}
	}
}

std::vector<int> Algorithms::getCutVector() {
	return optimal_cutvector_;
}

void Algorithms::reset() {
	lp_delete_model();
	lastIPscores_.clear();
	bdr_.clear();
	fdr_=0;
	optimal_cutvector_.clear();
	IPx_quantile_aprox_.clear();
	quantiles_available_ = false;
}

void Algorithms::UpdateBDR() {
	static int count=1; // For the unweighted average

	if(bdr_.size()==0){
		//First run
		bdr_.resize(Mopt_);
		for(int d=1;d<Mopt_;d++){
			bdr_.at(d)=(lastIPscores_.at(Mopt_-1)-lastIPscores_.at(Mopt_-d-1))/d;
		}
	}else{
		//Update
		if(bdr_update_coef_==0){ //Unweighted
			for(int d=1;d<Mopt_;d++){
				bdr_.at(d)=(count*bdr_.at(d) + (lastIPscores_.at(Mopt_-1)-lastIPscores_.at(Mopt_-d-1))/d)/(count+1);
			}
			count++;
		}
		else{ //Weighted
			for(int d=1;d<Mopt_;d++){
				bdr_.at(d)=(1-bdr_update_coef_)*bdr_.at(d) + (bdr_update_coef_)*(lastIPscores_.at(Mopt_-1)-lastIPscores_.at(Mopt_-d-1))/d;
			}
		}
	}
}

void Algorithms::UpdateFDR() {
	static float fdr_num;
	static float fdr_den;
	static int count=1; // For the unweighted average;

	if(fdr_==0){
		//First run
		fdr_num=(lastM-Mopt_)*(lastIPscores_.back()-lastIPscores_.at(Mopt_-1));
		fdr_den=(lastM-Mopt_)*(lastM-Mopt_);
		fdr_=fdr_num/fdr_den;
	}else{
		//Update
		if(fdr_update_coef_==0){ //Unweighted
			fdr_num = (count*fdr_num + (lastM-Mopt_)*(lastIPscores_.back()-lastIPscores_.at(Mopt_-1)))/(count+1);
			fdr_den = (count*fdr_den + (lastM-Mopt_)*(lastM-Mopt_))/(count+1);
			fdr_ = fdr_num/fdr_den;
			count++;
		}
		else{ // Weighted
			fdr_num= (1-fdr_update_coef_)*fdr_num + (fdr_update_coef_)*(lastM-Mopt_)*(lastIPscores_.back()-lastIPscores_.at(Mopt_-1));
			fdr_den= (1-fdr_update_coef_)*fdr_den + (fdr_update_coef_)*(lastM-Mopt_)*(lastM-Mopt_);
			fdr_=fdr_num/fdr_den;
		}
	}
}

int Algorithms::lp_matrix_formulation(vector<double>& c, vector<double>& p) {
	//Matrix formulation from the paper 'Real-time Distributed Visual Feature Extraction from Video in Sensor Networks'

	REAL *C =(REAL*)calloc(num_using_nodes_,sizeof(REAL));
	REAL *P =(REAL*)calloc(num_using_nodes_,sizeof(REAL));

	for(int i=0;i<num_using_nodes_;i++){
		C[i] = c.at(i);
		P[i] = p.at(i);
	}

	if(quantiles_available_==false){
		set_uniform_quantiles();
	}


/*	if(overlap_!=0){
		int nnodes=num_using_nodes_;
		num_using_nodes_=std::min( (int)std::floor(1/(2*overlap_)), nnodes);
			if(num_using_nodes_!=nnodes) fprintf(stderr, "Cannot use more than %d nodes when the overlap is %f\nUsing %d nodes instead.\n", num_using_nodes_, overlap_, num_using_nodes_);
	}
*/

	lp_D=(REAL*)calloc(num_using_nodes_*num_using_nodes_,sizeof(REAL));
	lp_G=(REAL*)calloc(num_using_nodes_,sizeof(REAL));
	lp_E=(REAL*)calloc(num_using_nodes_,sizeof(REAL));

	if(lp_D==NULL || lp_G==NULL || lp_E==NULL){
		return -1;
	}
	//lp_D, lp_G, and lp_E must contain all zeros before we start building the model matrix
	memset(lp_D, 0, (num_using_nodes_*num_using_nodes_)*sizeof(*lp_D));
	memset(lp_G, 0, (num_using_nodes_)*sizeof(*lp_G));
	memset(lp_E, 0, (num_using_nodes_)*sizeof(*lp_E));

	REAL *lp_CM=(REAL*)calloc(num_using_nodes_,sizeof(REAL));

	//Tidle: D1*xi + G1
	for(int m=1; m<num_using_nodes_; m++){
		lp_G[m] += height_*width_*overlap_*C[0];
		for(int j=1; j<m; j++){
			lp_G[m] += 2*height_*width_*overlap_*C[j];
		}
	}
	
	for(int m=0; m<num_using_nodes_; m++){
		for(int n=0; n<num_using_nodes_; n++){
			if(m==n+1){
				lp_D[m*num_using_nodes_+n] += height_*width_*C[n];
			}else if(m>n+1){
				lp_D[m*num_using_nodes_+n] += height_*width_*(C[n]-C[n+1]);
			}
		}
	}

	//Ttransmit (non-overlapping):
	for(int n=0; n<num_using_nodes_; n++){
		for(int m=0; m<num_using_nodes_; m++){
			if(n==m){
				lp_D[m*num_using_nodes_+n] += height_*width_*C[n];
			}else if(m==n+1){
				lp_D[m*num_using_nodes_+n] += -height_*width_*C[n+1];
			}
		}
	}

	lp_G[0] += height_*width_*overlap_*C[0];
	for(int n=1; n<num_using_nodes_-1; n++){
		lp_G[n] += 2*height_*width_*overlap_*C[n];
	}
	lp_G[num_using_nodes_-1] += height_*width_*overlap_*C[num_using_nodes_-1];
	

	//Tdetect (as function of the area): P = 1/Pdpx
	for(int n=0; n<num_using_nodes_; n++){
		for(int m=0; m<num_using_nodes_; m++){
			if(n==m){
				lp_D[m*num_using_nodes_+n] += P[n]*height_*width_;
			}
			else if(m==n+1){
				lp_D[m*num_using_nodes_+n] += -P[n+1]*height_*width_;
			}
		}
	}

	//E : Tdetect (as function of the number of interest points) + Textract (as function of the number of interest points):
		//P*alpha_d = 1/Pdip + 1/Pe
	for(int n=0; n<num_using_nodes_; n++){
		lp_E[n] += P[n]*alpha_d_;
	}

	free(lp_CM);

	free(C);
	free(P);
}

void Algorithms::lp_create_model() {
	int Ncol=3*num_using_nodes_+(num_using_nodes_-1)*(num_quantiles_+2);	//Ncol inicial
	int i,j;
	int *colno=(int*)malloc((Ncol+1)*sizeof(int)); // must be 1 more then number of columns ! //
	REAL *row=(REAL*)malloc((Ncol+1)*sizeof(REAL)); // must be 1 more then number of columns ! //
	int *sosvars=(int*)malloc((num_quantiles_+2)*sizeof(int));	//???
	int ret;

	lp = make_lp(0,Ncol);	//LP inicial con 0 filas
	if(lp == NULL) {
		fprintf(stderr, "Unable to create new LP model\n");
//		return(-1);
	}
	resize_lp(lp, 6*num_using_nodes_-3, Ncol);	//add rows

	//Set objective function
	set_obj(lp,num_using_nodes_+1,1);	//sets the objective value = 1

	//Add constraints:
	set_add_rowmode(lp, TRUE);	//If TRUE, then add_constraintex performs best.

	// D*x + E*ip - t <= -G
	for(i=0;i<num_using_nodes_;i++){
		for(j=0;j<num_using_nodes_;j++){
			colno[j]=POS_Xi(j);
			row[j]=lp_D[i*num_using_nodes_+j];
		}
		colno[j]=POS_IPi(i); //ip(i);
		row[j]=Mopt_*lp_E[i];
		colno[j+1]=POS_T;
		row[j+1]=-1;// -t
		add_constraintex(lp,j+2,row,colno,LE,-lp_G[i]);
	}

	//Increasing constraints, considering the overlap, (x[n]-x[n+1]<=-2*overlap_)
	for(i=0;i<num_using_nodes_-2;i++){
		colno[0]=POS_Xi(i);
		row[0]=1;
		colno[1]=POS_Xi(i+1);
		row[1]=-1;
		add_constraintex(lp,2,row,colno,LE,-2*overlap_);
	}
	colno[0]=POS_Xi(i);
	row[0]=1;
	colno[1]=POS_Xi(i+1);
	row[1]=-1;
	add_constraintex(lp,2,row,colno,LE,-overlap_);


	//And the last cut must be 1: (x[num_using_nodes_]=1)
	colno[0]=POS_Xi(num_using_nodes_-1);
	row[0]=1;
	add_constraintex(lp,1,row,colno,EQ,1);

	//And now we define the number of interest points for each cut (ip1, ip2,...)
	// ip1=f1
	// ip2=f2-f1
	// ip3=f3-f2
	// ipN=1-f3

	//ip1:
	colno[0]=POS_IPi(0); //ip1
	row[0]=-1;
	colno[1]=POS_Fi(0); //f1
	row[1]=1;
	add_constraintex(lp,2,row,colno,EQ,0);

	//ip2 to ip(N-1):
	for(i=1;i<num_using_nodes_-1;i++){
		colno[0]=POS_IPi(i); //ip(i)
		row[0]=-1;
		colno[1]=POS_Fi(i); //f(i)
		row[1]=1;
		colno[2]=POS_Fi(i-1); //f(i-1)
		row[2]=-1;
		add_constraintex(lp,3,row,colno,EQ,0);
	}

	//ipN:
	colno[0]=POS_IPi(num_using_nodes_-1); //ipN
	row[0]=1;
	colno[1]=POS_Fi(num_using_nodes_-2); //f(N-1)
	row[1]=1;
	add_constraintex(lp,2,row,colno,EQ,1);


	//SOS variables:
	// d10+d11+d12+d13+...=1
	for(i=0;i<num_using_nodes_-1;i++){
		for(j=0;j<num_quantiles_+2;j++){
			colno[j]=POS_Dij(i,j);
			row[j]=1;
			sosvars[j]=POS_Dij(i,j);
		}
		char sosName[] = "SOS"; //Gives a warning otherwise...
		add_SOS(lp, sosName, 2, 1, num_quantiles_+2, sosvars, NULL);
		add_constraintex(lp,num_quantiles_+2,row,colno,EQ,1);
	}

	//So now we define the piecewise functions f (number of interest points left of x)
	for(i=0;i<num_using_nodes_-1;i++){
		colno[0]=POS_Fi(i); //fi
		row[0]=-1;
		for(j=0;j<num_quantiles_;j++){
			colno[j+1]=POS_Dij(i,j+1);
			row[j+1]=(j+1.0)/num_quantiles_;
		}
		colno[j+1]=POS_Dij(i,j+1);
		row[j+1]=1;
		add_constraintex(lp,num_quantiles_+2,row,colno,EQ,0);
	}

	// Now x1=q1*d11 + q2*d12 + ...
	for(i=0;i<num_using_nodes_-1;i++){
		colno[0]=POS_Xi(i);
		row[0]=-1;
		for(j=0;j<num_quantiles_;j++){
			colno[j+1]=POS_Dij(i,j+1);
			row[j+1]=(float)IPx_quantile_aprox_.at(j)/width_;
		}
		colno[j+1]=POS_Dij(i,j+1);
		row[j+1]=1;
		add_constraintex(lp,num_quantiles_+2,row,colno,EQ,0);
	}

	set_add_rowmode(lp, FALSE);

	free(colno);
	free(row);
	free(sosvars);
	is_lpmodel_created_=true;
}

void Algorithms::set_uniform_quantiles() {
	IPx_quantile_aprox_.resize(num_quantiles_);
	for(int i=0; i<num_quantiles_; i++){
		IPx_quantile_aprox_.at(i)=width_*(i+1)/num_quantiles_;
	}
}

int Algorithms::lp_solve_model() {
	//Returns 0 when an optimal solution is found,
	//returns 1 if the solution is suboptimal due to an overlap constraint, and we should use fewer nodes
	//returns 2 when a suboptimal solution is found (solver timed out)
	//returns 3 if another error occurred

	int ret;
	int Ncol=3*num_using_nodes_+(num_using_nodes_-1)*(num_quantiles_+2);
	REAL *vars = (REAL*)malloc(Ncol*sizeof(REAL));

	set_verbose(lp, IMPORTANT);	//reports information back to the user. How much information is reported depends on the verbose level
	set_timeout(lp, solver_timeout_);	//sets a timeout in seconds
	default_basis(lp);	//sets the starting base to an all slack basis

	ret = solve(lp);	
	if(ret==1){
		fprintf(stderr, "lp_solve: Suboptimal solution (solver timed out)\n");
		return 2;
	}
	if(ret!=0 && ret!=1){
		fprintf(stderr, "lp_solve: Solution not found\n");
		return 3;
	}

	print_solution(lp, 3);	//prints the solution (variables) of the lp
	get_variables(lp, vars);	//retrieve the values of the variables

	//Cutvector:
	optimal_cutvector_.resize(num_using_nodes_);
	for(int i=0;i<num_using_nodes_;i++){
		optimal_cutvector_.at(i)=round(width_*vars[i]);
	}
	//Estimated completion time:
	est_completion_time_=(float)vars[num_using_nodes_];

/*	printf("\nOptimal cutvector:\n");
	for(int i=0;i<num_using_nodes_;i++){
		printf("%d\n", optimal_cutvector_.at(i));
	}

	printf("Estimated completion time: %f\n", est_completion_time_);
*/

	//If the first cut is 0, that would be an indication that the cutvector we obtained is suboptimal,
	//due to the overlap constraint.
	//We should most likely use fewer nodes.
	if(optimal_cutvector_.front()==0){
		fprintf(stderr, "lp_solve: Suboptimal solution (first cut was 0), we should use fewer processing nodes\n");
		return 1;
	}

	free(vars);
	return ret;
}

void Algorithms::setInitialDetectionThreshold(double th) {
	lastThreshold_ = th;
}

float Algorithms::getCompletionTime() {
	return est_completion_time_;
}

void Algorithms::lp_delete_model() {
	free(lp_D); free(lp_G); free(lp_E);
	if(is_lpmodel_created_==true) free_lp(&lp);
	lp_D=NULL;
	lp_G=NULL;
	lp_E=NULL;
	is_lpmodel_created_=false;
}
