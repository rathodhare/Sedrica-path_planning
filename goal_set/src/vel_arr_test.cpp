#include <bits/stdc++.h>
#include <fstream>
#include <algorithm>
#include <math.h>

using namespace std;

double v_in=0,v_out=0; //initial velocity
double f_acc=1.5,r_acc = 1.1; //Reverse acc to be tuned
double f_decc=3.5,r_decc = 4; //Reverse decc to be tuned
double f_c_acc = 1.7,r_c_acc = 1.3; //Reverse c_acc to be tuned
double f_v_max = 5.55,f_v_zebra = 2.77,r_v_max = 3,r_v_zebra = 1.5; //Reverse values to be tuned
double weight = 0.7;
bool output_vel;

int velD_curr_can=0;
float velD_curr=velD_curr_can*5/18;

float tr,tr_dir;
std::vector<float> ze,v,v_copy;

/*void output_vel_init(const std_msgs::Bool::ConstPtr& x){
	output_vel = x->data;
	//cout<<" output_vel "<<output_vel<<endl;
  	if(output_vel)
  	v_out = 5.55;
  	else v_out = 0;
}
void traffic_init(const std_msgs::Float64::ConstPtr& tra){
	//	cout<<"yo_tr";
	tr = (tra->data);
}

void traffic_dir_init(const std_msgs::Float64::ConstPtr& tra){
	//cout<<"called!!";
	tr_dir = (tra->data);
}

void zebra_init(const std_msgs::Float64MultiArray::ConstPtr& zeb){
	//cout<<"zebra_init";
	ze.clear();
	for(int i=0;i<zeb->data.size();i++)
		ze.push_back(zeb->data[i]);
}*/

vector<double> vmax(double c[], int n){
   vector<double> v_m;
	 v_m.push_back(v_in);
   for(int i=0; i<n; i++){
   	if(c[i]==-4000)
   		v_m.push_back(0);
   	else if(c[i]==2002)
   		v_m.push_back(f_v_zebra);
   	else if(c[i]==-2002)
   		v_m.push_back(r_v_zebra);
    else if(c[i]>=2000)
      	v_m.push_back(f_v_max);
    else if(c[i]>=0)
      	v_m.push_back(sqrt(f_c_acc*c[i]));
  	else
  		v_m.push_back(-sqrt(f_acc*c[i]));
  //cout<<v_m[i]<<endl;
  }
 	v_m.push_back(v_out);
  //cout<<v_m[v_m.size()-1]<<endl;
	return v_m;
}

vector<vector<double> > segment(vector<double> v){
  int n=0,ctr = 0;
  vector<vector<double> > x;
  for(int i=0;i<v.size()-1;i++){
  	vector<double> row;
  	while(i<v.size()-1 && v[i]>=v[i+1]){
  		row.push_back(v[i]);
  		i++;
  		ctr = 1;
  	}
  	while(i<v.size()-1 && v[i]<=v[i+1] && ctr!=1){
  		row.push_back(v[i]);
  		i++;
  		ctr = 2;
  	}
  	ctr=0;
  	row.push_back(v[i]);
  	x.push_back(row);
  }
  return x;
}

void smooth(vector<double> &v_m, vector<double> &distance){
	for(int i=1;i<v_m.size();i++){
		if((v_m[i-1]-v_m[i])>1){
			for(int j=i; j>=0; j--){
				double val2 = sqrt( v_m[i]*v_m[i]+2*f_acc*(distance[i]-distance[j]) ) ;	
//			cout<<"val="<<val2<<"		"<<"v["<<j<<"]="<<v_m[j]<<"			"<<distance[i]-distance[j]<<endl;
				v_m[j]=min(val2,v_m[j]);}
		}
	}
}

void path_vel(vector<vector <int>  > msg){
//cout<<" entered path_vel"<<endl;
	//cout<<"empty check"<<endl;
	int no_data=msg.size();
 // double x[no_data], y[no_data];
  	double r[no_data-2];
 	v.push_back(v_in); //initial condition
 	///v[no_data-3]=v_out;
	vector<double> s;
  	vector<double> v_m;
  	vector<vector<double> > segment_v_m;


	double z1,z2;
	for(int i=no_data-2,k=0; i>=1; i--,k++){
		//cout<<" checked path 2"<<endl;

		double x1 = msg[i-1][0] ,x2 = msg[i][0], x3 = msg[i+1][0];
		double y1 = msg[i-1][1] ,y2 = msg[i][1], y3 = msg[i+1][1];
		z2 = msg[i-1][2], z1 = msg[i][2];
		double a=(x1*(y2-y3)-y1*(x2-x3)+x2*y3-y2*x3);
		double b=((x1*x1+y1*y1)*(y3-y2)+(x2*x2+y2*y2)*(y1-y3)+(x3*x3+y3*y3)*(y2-y1));
		double c=((x1*x1+y1*y1)*(x2-x3)+(x2*x2+y2*y2)*(x3-x1)+(x3*x3+y3*y3)*(x1-x2));
		double d=((x1*x1+y1*y1)*(x3*y2-x2*y3) + (x2*x2+y2*y2)*(x1*y3-x3*y1) + (x3*x3+y3*y3)*(x2*y1-x1*y2));
	//cout<<" checked path 3"<<endl;
	//cout<<"a"<<(double)a<<"b"<<(double)b<<"e"<<(double)e<<endl;
		if(a!=0)
			r[k]=sqrt((b*b+c*c-4*a*d)/(4*a*a));
		if(a==0)
			r[k]=2000;
	//cout<<"radius "<<r[k]<<"\n";
		s.push_back(0.0); 
		for(int j=1; j<no_data; j++)
    		s.push_back(s[j-1]+sqrt(((x2-x1)*(x2-x1))+((y2-y1)*(y2-y1))));
	}
	int ctr=0;
	for(int i=1;i<no_data-2;i++){
		r[i] = weight*r[i] + (1-weight)*r[i-1];
	}
	for(int i=0;i<no_data-2;i++){
		if(r[i]>(f_v_max*f_v_max/f_c_acc))
			r[i]=2000;
		//cout<<r[i]<<endl;
	if((z1==1 && z2==0) || (z2==1 && z1==0))
		r[i] = -4000;
	else if(z2==1)
		r[i] = -r[i];
	}
	//cout<<"Yos"<<ctr;
	//Traffic Light Detection
	int pos = no_data-1;
	for(int i=0;i<no_data-1;i++){
		if(tr>=(s[i]+7) && tr<=(s[i]+8) && tr_dir==1){
			r[i]=-4000;
			pos = i;
			break;
		}
	}
	if(tr>=0 && tr<=7)
		pos = 0;
	for(int i=pos;i<no_data-2;i++)
		r[i] = -4000;
	//Zebra Crossing
	for(int i=0;i<ze.size();i++){
		if(ze[i]!=-1){
			for(int j=0;j<no_data-2;j++){
				if( s[j]==ze[i] )
					r[j]=2002;
			}
			//cout<<"YOS!\n";
		}
	}

 // for(int i=0;i<47;i++) cout<<s[i]<< " ";cout<<endl;
 	v_m=vmax(r, no_data-2);
 	//v_m.push_back(v_out);
	//for(int j=0;j<v_m.size();j++) 
		//cout<<"vmax = "<<v_m[j]<<endl;
 	segment_v_m=segment(v_m);
 	//cout<<"Segment size: "<<segment_v_m.size()<<endl;
	smooth(v_m, s);

  //for(int i=0;i<segment_v_m.size();i++){
    //   for(int j=0;j<segment_v_m[i].size();j++) cout<<segment_v_m[i][j]<<" ";cout<<endl;}

  int a=0;
  int b=0;
  for(int i=0; i<segment_v_m.size();i++ ){
	//cout<<"a"<<a<<" 		"<<b<<endl;
		for(int j=0; j<segment_v_m[i].size(); j++){
			if(!(i==0 && j==0)){
    	double val1 = sqrt(v[b]*v[b]+2*f_acc*(s[a]-s[b]));
      double val2 = sqrt( v_m[b+segment_v_m[i].size()-1]*v_m[b+segment_v_m[i].size()-1]+2*f_decc*(s[b+segment_v_m[i].size()-1]-s[a]) ) ;
      double temp = min((double)v_m[a],(double)val1);
      //cout<<a<<"  "<<b<<"  "<<min(val2,temp)<<" ";
      //cout<<"val1="<<val1<<"      "<<"val2="<<val2<<"      "<<"vmax"<<v_m[a-1]<<'\n';
      v.push_back(min((double)temp, (double)val2));
      }
      a++;
		}
 		b=b+segment_v_m[i].size();
		v[b]=v[b-1];
	}
	
	for(int i=0; i<msg.size()-2; i++){
		std::cout<<v[i]<<'\n';}
	velD_curr=velD_curr_can*5/18;
	v_in=velD_curr;
	v_copy = v;
    v.clear();
	
}

int main(int argc, char **argv)
{
	//In the x vector the fields are as follows: 1. x 2. y 3. traffic sense (+1 or -1) 4.zebra sense 5.Distance to traffic lights (distance or -1)
  tr = -1;
  tr_dir = -1;
  ze.push_back(-1);
  vector< vector <int> > msg;
  for(int i=0;i<10;i++){
  	msg[i][0]=i;
  	msg[i][1]=0;
  	msg[i][2]=0;
  }
  for(int i=0;i<10;i++){
  	msg[i][0]=10-i;
  	msg[i][1]=0;
  	msg[i][2]=1;
  }
  path_vel(msg);
  return 0;
}
