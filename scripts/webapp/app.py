from asyncore import dispatcher
from flask import Flask, render_template, request, redirect, url_for
# from sqlalchemy import create_engine
# from sqlalchemy.orm import sessionmaker
# from database_setup import Base, RobotStat
from flask_wtf import FlaskForm, CSRFProtect
from flask_bootstrap import Bootstrap5, SwitchField
from flask_sqlalchemy import SQLAlchemy
from datetime import datetime
import logging
from robot_web_client import RobotWebClient
from threading import Thread
import rclpy
import time

app = Flask(__name__)
app.secret_key = 'dev'

app.config['SQLALCHEMY_TRACK_MODIFICATIONS'] = False
app.config['SQLALCHEMY_DATABASE_URI'] = 'sqlite:///:memory:'

bootstrap = Bootstrap5(app)
db = SQLAlchemy(app)
csrf = CSRFProtect(app)

rclpy.init(args=None)
robot_client = RobotWebClient()

# TODO: move this class to a separate file 
class RobotStat(db.Model):
    __tablename__ = 'robot-status'

    id = db.Column(db.Integer, primary_key=True)
    robot_stat_id = db.Column(db.Integer, nullable=False) # uuid of robot status entry
    robot_stat_time = db.Column(db.String(50), nullable=False) # time of the status sent 
    robot_active = db.Column(db.Integer, nullable=False) # 0 inactive, 1 active
    robot_ctrl_mode = db.Column(db.String(50), nullable=False) # 0 manual mode, 1 autonomous mode
    robot_battery = db.Column(db.String(50), nullable=False) # battery percentage
    robot_speed = db.Column(db.String(100), nullable=False) # speed time and location
    robot_in_row = db.Column(db.String(50), nullable=False) # 0: robot in row, should spray; 1: outside of row, should not spray
    robot_temp_pc = db.Column(db.String(50), nullable=False) # 0: robot in row, should spray; 1: outside of row, should not spray
    robot_temp_enclosure = db.Column(db.String(50), nullable=False) # 0: robot in row, should spray; 1: outside of row, should not spray
    # robot_location_x = Column(String(50), nullable=False) # robot gps location first coordinate 
    # robot_location_y = Column(String(50), nullable=False) # robot gps location second coordinate
    # robot_task_stat = Column(Integer, nullable=False) # 0 no task, 1 active task, 2 pending task, 3 paused task
    # robot_task_name = Column(String(50), nullable=False) # name of the task dispatched to the robot
    # robot_log = Column(String(300), nullable=False) # additional log info 

    @property
    def serialize(self):
        return {
            'robot_stat_id': self.robot_stat_id,
            'robot_stat_time': self.robot_stat_time,
            'robot_active': self.robot_active,
            'robot_ctrl_mode': self.robot_id,
            'robot_battery': self.robot_battery,
            'robot_speed': self.robot_speed,
            'robot_in_row': self.robot_in_row,
            'robot_temp_pc': self.robot_temp_pc,
            'robot_temp_enclosure': self.robot_temp_enclosure,
            # 'robot_location': self.robot_location,
            # 'robot_log_time': self.robot_log_time,
        }


@app.before_first_request
def before_first_request_func():
    db.drop_all()
    db.create_all()
    
    m = RobotStat(robot_stat_id="xxx", 
                    robot_stat_time="xx:xx:xx EST xx/xx/xx", 
                    robot_active=0, 
                    robot_ctrl_mode="xx",
                    robot_battery="xx%",
                    robot_speed="xxxx",
                    robot_in_row="xxxx",
                    robot_temp_pc="xxxx",
                    robot_temp_enclosure="xxxx",
                    )
    db.session.add(m)
    db.session.commit()


@app.route('/')
def index():
    # if show_robot_status is pressed, read latest robot status entry
    print("in index")
    robot = db.session.query(RobotStat).first()
    return render_template("index.html", robot=robot)

@app.route("/start_row_task/")
def start_robot():
    robot_client.setStartAutoFlag(True)
    print("Start")
    return '', 204

@app.route("/stop_row_task/")
def stop_robot():
    robot_client.setStartAutoFlag(False)
    print("Stop")
    return '', 204

# @app.route("/show_status/", methods=['POST'])
@app.route("/show_status/")
def show_robot_status():

    # get data from robot client 
    gps = robot_client.getGPS()
    speed = robot_client.getSpeed()
    battery = robot_client.getBattery()
    in_row_stat = robot_client.getInRowStat()
    auto_mode_flag = robot_client.getAutoMode()
    control_mode = ""
    if auto_mode_flag:
        control_mode = "Autonomous"
    else:
        control_mode = "Tele-op"

    pc_temp = robot_client.getTempPC()
    enclosure_temp = robot_client.getTempEnclosure()


    now = datetime.now()
    dt_string = now.strftime("%H:%M:%S EST %Y/%m/%d")
    
    # create an entry in db 
    m = RobotStat(robot_stat_id="husky", 
                        # robot_stat_time='03:24 EST, 04/01/2022', 
                        robot_stat_time=dt_string, 
                        robot_active=0, 
                        robot_ctrl_mode=control_mode,
                        robot_battery=battery,
                        robot_speed=speed,
                        robot_in_row=in_row_stat,
                        robot_temp_pc=pc_temp,
                        robot_temp_enclosure=enclosure_temp,
                        )
    db.session.add(m)
    db.session.commit()
    robot = db.session.query(RobotStat).filter_by(robot_stat_time=dt_string).first()
    # robot = db.session.query(RobotStat).first()
    # robot = db.session.query.filter_by(robot_stat_id='husky').first()
    # logging.warning(robot.robot_stat_time)
    print("-----")
    print(robot.robot_stat_time)
    # print(speed)
    # print(gps)
    
    # display on app
    return render_template('index.html', robot=robot)

# @app.route('/',methods=['GET','POST'])
# def updateRobotStat():
#     # if start/stop button is pressed, create a new robot status  

#     if request.method == 'POST':
#        newBook = Book(title = request.form['name'], author = request.form['author'], genre = request.form['genre'])
#        session.add(newBook)
#        session.commit()
#        return redirect(url_for('showBooks'))
#     else:
#        return render_template('newBook.html')


def web_server_spin():
    while rclpy.ok():
        rclpy.spin_once(robot_client)
        time.sleep(0.2)

if __name__ == '__main__':

    # create robot instances in database 
    # rbstatOne = RobotStat(robot_stat_id="husky", 
    #                     robot_stat_time="03:24 EST, 04/01/2022", 
    #                     robot_active=0, 
    #                     robot_ctrl_mode="manual")
    # session.add(rbstatOne )
    # session.commit()

    spin_thread = Thread(target=web_server_spin, args=())
    spin_thread.start()

    app.run(host='0.0.0.0', port=4998)

