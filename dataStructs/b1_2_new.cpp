// Publisher-Master-Subscriber Implementation in C++
// Publisher sends message to topic listed on master, master forwards it to the registered subscribers

#include <iostream>
#include <string>
#include <cstring>
#include <utility>
#include <vector>

using namespace std;

///////////////////////////// Publisher 

class Publisher {
	string topic;
	void (*add_callback) (Publisher *,  string);
	void (*del_callback) (Publisher *);
	void (*pub_callback) (string, string);

	public:
	Publisher(string, void (Master::*)(Publisher *, string), void(Master::*)(Publisher *), void(Master::*)(string, string));
	~Publisher();

	void publish(string);
};

Publisher::Publisher(string tpc, void (Master::*add_cb)(Publisher *, string), void (Master::*del_cb)(Publisher *), void (Master::*pub_cb)(string, string)) { // Construct publisher with pointer to master and a string topic
	cout << "\nEntered publisher constrcut";
	topic = tpc;
	cout << "\n2";
	add_callback = add_cb;
	cout << "\n3";
	del_callback = del_cb;
	cout << "\n4";
	pub_callback = pub_cb;
	cout << "\n5";
	cout << "\n6";
	(*add_callback)(this, topic); // Add this publisher to master's publisher-topic list
	cout << "\n finished adding pub to master";
}

Publisher::~Publisher() {
	(*del_callback)(this); // Remove this publisher from master's publisher-topic list
}

void Publisher::publish(string msg) {
	(*pub_callback)(msg, topic); // Call master's routing function to forward message to the correct subscribers
}

/////////////////////////////// Subscriber
class Subscriber {
	string topic;
	string response; // Stores the message received on topic
	void (*add_callback) (Subscriber *, string);
	void (*del_callback) (Subscriber *);
	
	public:
	Subscriber(string, void(*)(Subscriber *, string), void(*)(Subscriber *));
	~Subscriber();

	void store_response(string);
	void display_store();
};

Subscriber::Subscriber(string tpc, void (*add_cb)(Subscriber *, string), void (*del_cb)(Subscriber *)) {
	topic = tpc;
	add_callback = add_cb;
	del_callback = del_cb;
	cout << "\nMiddle of sub construct";
	(*add_callback)(this, topic); // Add this subscriber to master's subscriber-topic list
	cout << "\nsub says successfully added to master";
}

Subscriber::~Subscriber() {
	(*del_callback)(this); // Remove this subscriber from master's subscriber-topic list
}

void Subscriber::store_response(string msg) {
	response = msg;
}

void Subscriber::display_store() {
	cout << "Subscriber stores " << response;
}

///////////////////////////////// Master
class Master {
	
	vector<pair<Publisher*, string>> pub_list;
	vector<pair<Subscriber*, string>> sub_list;
	
	public:
	void add_pub(Publisher*, string);
	void del_pub(Publisher*);
	void add_sub(Subscriber*, string);
	void del_sub(Subscriber*);
	void route(string, string);
};

void Master::add_pub(Publisher* pub, string topic) {
	pub_list.push_back(make_pair(pub, topic));
}

void Master::del_pub(Publisher* pub) {
	for (int i = 0; i < pub_list.size(); ++i) { // Search for the location of publisher, then erase
		if (pub_list[i].first == pub) {
			pub_list.erase(pub_list.begin() + i);
			break;
		}
	}
}

void Master::add_sub(Subscriber* sub, string topic) {
	sub_list.push_back(make_pair(sub, topic));
}

void Master::del_sub(Subscriber* sub) {
        for (int i = 0; i < sub_list.size(); ++i) { // Search for the location of subscriber, then erase
                if (sub_list[i].first == sub) {
                        sub_list.erase(sub_list.begin() + i);
                        break;
                }
        }	
}

void Master::route(string msg, string topic) { // Search for all subscribers of topic and call their respective store functions
	for (int i = 0; i < sub_list.size(); ++i) {
		if (sub_list[i].second == topic) {
			sub_list[i].first->store_response(msg);
		}
	}
}

/*

void add_pub_helper(Publisher *pub, int *m, string topic) {
	Master *mstr = (Master *)m;
	mstr->add_pub(pub, topic);
}

void del_pub_helper(Publisher *pub, int *m) {
	Master *mstr = (Master *)m;
	mstr->del_pub(pub);
}

void publish_helper(int *m, string msg, string topic) {
	Master *mstr = (Master *)m;
	mstr->route(msg, topic);
}

void add_sub_helper(Subscriber *pub, int *m, string topic) {
	Master *mstr = (Master *)m;
	mstr->add_sub(pub, topic);
}

void del_sub_helper(Subscriber *pub, int *m) {
	Master *mstr = (Master *)m;
	mstr->del_sub(pub);
}
*/

int main() {
	Master *mstr;

	string topic1 = "vodka-boil", topic2 = "ilate-gay", message = "discuss something better";
	Publisher pub1(topic1, &(mstr->add_pub), &(mstr->del_pub), &(mstr->route));
	Subscriber sub1(topic1, &(mstr->add_sub), &(mstr->del_sub));
	Subscriber sub2(topic1, &(mstr->add_sub), &(mstr->del_sub));

	sub1.display_store();
	pub1.publish(message);
	sub1.display_store();
	sub2.display_store();

	return 0;
}
