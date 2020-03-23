#include <iostream>
#include <utility>
#include <vector>
#include <string>
#include <cstring>

class Master;
class Publisher;
class Subscriber;

using namespace std;

////////////////////////////////////////////////////////////////////

class Subscriber {
	string topic;
	string store;
	void (*del_sub_h)(int *m, Subscriber* sub);
	int *mstr;
	
	public:
	Subscriber(int *m, string tpc, void (*a_s_h)(int *m, Subscriber * sub, string tpc), void (*d_s_h)(int *m, Subscriber * sub));
	~Subscriber();
	void update(string value);
	void display();
};

Subscriber::Subscriber(int *m, string tpc, void (*a_s_h)(int *m, Subscriber * sub, string tpc), void (*d_s_h)(int *m, Subscriber * sub)) {
	mstr = m;
	topic = tpc;
	store = "";
	del_sub_h = d_s_h;
	a_s_h(m, this,tpc); 
}

Subscriber::~Subscriber() {
	del_sub_h(mstr, this);
}

void Subscriber::update(string value) {
	store = value;
}

void Subscriber::display() {
	cout << "I store " << store << "\n";
}

////////////////////////////////////////////////////////////////////

class Publisher {
	string topic;
	int *mstr;
	void (*del_pub_h)(int *m, Publisher* pub);
	void (*pub_pub_h)(int *m, string tpc, string val);

	public:
	Publisher(int *m, string tpc, void (*a_p_h)(int *m, Publisher * pub, string tpc), void (*d_p_h)(int *m, Publisher * pub), void(*p_p_h)(int *m, string tpc, string val));
	~Publisher();
	void publish(string value);
};

Publisher::Publisher(int *m, string tpc, void (*a_p_h)(int *m, Publisher * pub, string tpc), void (*d_p_h)(int *m, Publisher * pub), void(*p_p_h)(int *m, string tpc, string val)) {
        mstr = m;
        topic = tpc;
        del_pub_h = d_p_h;
	pub_pub_h = p_p_h;
        a_p_h(m, this, tpc);
}

Publisher::~Publisher() {
        del_pub_h(mstr, this);
}

void Publisher::publish(string value) {
	pub_pub_h(mstr, topic, value);
}

////////////////////////////////////////////////////////////////////

class Master {
	vector<pair<Subscriber*, string>> sub_list;
	vector<pair<Publisher*, string>> pub_list;

	public:
	void add_sub(Subscriber * sub, string tpc);
	void del_sub(Subscriber * sub);
	void add_pub(Publisher * pub, string tpc);
        void del_pub(Publisher * pub);
	void route(string tpc, string val);
};

void Master::add_sub(Subscriber * sub, string tpc) {
	sub_list.push_back(make_pair(sub, tpc));
}

void Master::del_sub(Subscriber * sub) {
	for (int i = 0; i < sub_list.size(); ++i) { // Search for the location of subscriber, then erase
                if (sub_list[i].first == sub) {
                        sub_list.erase(sub_list.begin() + i);
                        break;
                }
        }
}

void Master::add_pub(Publisher * pub, string tpc) {
        pub_list.push_back(make_pair(pub, tpc));
}

void Master::del_pub(Publisher * pub) {
        for (int i = 0; i < pub_list.size(); ++i) { // Search for the location of publisher, then erase
                if (pub_list[i].first == pub) {
                        pub_list.erase(pub_list.begin() + i);
                        break;
                }
        }
}

void Master::route(string tpc, string val) {
	for (int i = 0; i < sub_list.size(); ++i) {
                if (sub_list[i].second == tpc) {
                        sub_list[i].first->update(val);
                }
        }

}

//////////////////////////////////////////////////////////////////

void add_sub_helper(int *m, Subscriber * sub, string tpc) {
	Master *temp = (Master*)m;
	temp->add_sub(sub, tpc);
}

void del_sub_helper(int *m, Subscriber * sub) {
	Master *temp = (Master*)m;
	temp->del_sub(sub);
}

void add_pub_helper(int *m, Publisher * pub, string tpc) {
	Master *temp = (Master*)m;
        temp->add_pub(pub, tpc);
}

void del_pub_helper(int *m, Publisher * pub) {
	Master *temp = (Master*)m;
        temp->del_pub(pub);
}

void pub_pub_helper(int *m, string tpc, string val) {
	Master *temp = (Master*)m;
        temp->route(tpc, val);
}

/////////////////////////////////////////////////////////////////

int main() {
	void (*a_s_h)(int *m, Subscriber * sub, string tpc) = add_sub_helper;
	void (*d_s_h)(int *m, Subscriber * sub) = del_sub_helper;
	void (*a_p_h)(int *m, Publisher * pub, string tpc) = add_pub_helper;
        void (*d_p_h)(int *m, Publisher * sub) = del_pub_helper;
	void (*p_p_h)(int *m, string tpc, string val) = pub_pub_helper;
	
	Master maestro;
	string topic1 = "apples";
	string msg = "Keep Doc away";
	int *mstr1 = (int*)&maestro;
	
	Subscriber sub1(mstr1, topic1, a_s_h, d_s_h);
	sub1.display();
	Publisher pub1(mstr1, topic1, a_p_h, d_p_h, p_p_h);
	pub1.publish(msg);
	sub1.display();
	return 0;
}
