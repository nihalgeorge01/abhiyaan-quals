// Publisher-Master-Subscriber Implementation in C++
// Publisher sends message to topic listed on master, master forwards it to the registered subscribers

#include <iostream>
#include <string>
#include <cstring>
#include <utility>
#include <vector>

using namespace std;

class Master;
class Publisher;
class Subscriber;
///////////////////////////// Publisher

class Publisher {
	string topic;
	//Master *mstr;
	
	public:
	Publisher(Master *, string);
	~Publisher();

	void publish(Master *, string);
};

Publisher::Publisher(Master *m, string tpc) { // Construct publisher with pointer to master and a string topic
	topic = tpc;
	//mstr = m;
	m->add_pub(this, topic); // Add this publisher to master's publisher-topic list
}
/*
Publisher::~Publisher() {
	mstr->del_pub(this); // Remove this publisher from master's publisher-topic list
}
*/
void Publisher::publish(Master *mstr, string msg) {
	mstr->route(msg, topic); // Call master's routing function to forward message to the correct subscribers
}

/////////////////////////////// Subscriber

class Subscriber {
	string topic;
	string response; // Stores the message received on topic
	//Master *mstr;

	public:
	Subscriber(Master *, string);
	~Subscriber();

	void store_response(string);
	void display_store();
};

Subscriber::Subscriber(Master *m, string tpc) {
	topic = tpc;
	//mstr = m;
	m->add_sub(this, topic); // Add this subscriber to master's subscriber-topic list
}
/*
Subscriber::~Subscriber() {
	mstr->del_sub(this); // Remove this subscriber from master's subscriber-topic list
}
*/
void Subscriber::store_response(string msg) {
	response = msg;
}

void Subscriber::display_store() {
	cout << "Subscriber " << this << " stores " << response;
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

int main() {
	Master *m;
	string topic1 = "vodka-boil", topic2 = "ilate-gay", message = "discuss something better";
	Publisher pub1(m, topic1);
	Subscriber sub1(m, topic1);
	Subscriber sub2(m, topic1);

	sub1.display_store();
	pub1.publish(m, message);
	sub1.display_store();
	sub2.display_store();

	return 0;
}
