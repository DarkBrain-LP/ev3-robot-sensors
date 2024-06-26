import paho.mqtt.client as mqtt

class IntersectionServer:
    def __init__(self, broker_address):
        self.broker_address = broker_address
        self.passage_list = []
        self.attente_list = []
        self.client = mqtt.Client("Server")
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.connect(self.broker_address)

    def on_connect(self, client, userdata, flags, rc):
        print(f"Connected with result code {rc}")
        client.subscribe("intersection/request")
        client.subscribe("intersection/exit")

    def on_message(self, client, userdata, msg):
        topic = msg.topic
        message = msg.payload.decode()
        robot_id = message
        print(robot_id)

        if topic == "intersection/request":
            if robot_id not in self.passage_list:
                self.passage_list.append(robot_id)
            if robot_id == self.passage_list[0]:
                self.client.publish(f"intersection/response/{robot_id}", "GRANTED")
            else : 
                self.client.publish(f"intersection/response/{robot_id}", "WAIT")

        elif topic == "intersection/exit":
            if self.passage_list != []:
                if self.passage_list[0] == robot_id :
                    self.passage_list.pop(0)
                    self.client.publish(f"intersection/response/{robot_id}", "REMOVED")
                    # Si il y a des robots en attente, autoriser le 1er à passer
                    if self.passage_list != [] :
                        self.client.publish(f"intersection/response/{self.passage_list[0]}", "GRANTED")
                else :
                    self.client.publish(f"intersection/response/{robot_id}", "ERROR")

    def run(self):
        self.client.loop_forever()

if __name__ == "__main__":
    server = IntersectionServer("192.168.119.39")
    server.run()