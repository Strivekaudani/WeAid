from kivymd.app import MDApp
from kivymd.icon_definitions import md_icons
from kivy.lang import Builder
from kivy.uix.widget import Widget
from kivy.uix.screenmanager import ScreenManager, Screen
from kivy.uix.image import Image
from kivy.graphics.texture import Texture
from kivy.properties import ObjectProperty, StringProperty
from kivy_garden.mapview import MapView, MapMarker
from kivy.core.window import Window
from passlib.hash import sha256_crypt
from kivy.utils import platform
from kivymd.toast import toast
from kivy.clock import Clock, mainthread
from kivy.logger import Logger
from plyer import battery, tts, vibrator, accelerometer, gps, uniqueid, camera
from os.path import exists, join
import sqlite3

import cv2
import tensorflow as tf
from layers import L1Dist
import os
import numpy as np

if platform not in ["android", "ios"]:
    Window.size = (350, 580)

if platform == 'android':
    from jnius import autoclass
    import jnius

    BluetoothAdapter = autoclass('android.bluetooth.BluetoothAdapter')
    BluetoothDevice = autoclass('android.bluetooth.BluetoothDevice')
    BluetoothSocket = autoclass('android.bluetooth.BluetoothSocket')
    InputStreamReader = autoclass('java.io.InputStreamReader')
    BufferedReader = autoclass('java.io.BufferedReader')
    UUID = autoclass('java.util.UUID')
    System = autoclass('java.lang.System')

class SignUp(Screen):
    def signup(self):
        conn = sqlite3.connect('navigation.db')
        c = conn.cursor()

        email = self.ids.email.text
        name = self.ids.full_name.text
        password = self.ids.password.text
        confirm_password = self.ids.confirm_password.text
        hpassword = sha256_crypt.hash(str(password))

        e_mail = c.execute("SELECT email FROM users WHERE email = :email", {"email":email}).fetchone()

        if password == confirm_password:

            if e_mail is None:

                c.execute("INSERT into users(email, name, hpassword) VALUES (:email, :name, :hpassword)",
                    {
                        'email': email, 'name': name, 'hpassword':hpassword
                    })

                conn.commit()
                conn.close()

            else:
                self.ids.welcome_label.text = f'Email already exists'
                self.ids.email.text = ''

        else:
            self.ids.welcome_label.text = f'Password does not match'
            self.ids.password.text = ''
            self.ids.confirm_password.text = ''

class Login(Screen):

    def signin(self):
        conn = sqlite3.connect('navigation.db')
        c = conn.cursor()

        email = self.ids.email.text
        password = self.ids.password.text

        email_data = c.execute("SELECT email FROM users WHERE email = :email", {"email":email}).fetchone()
        password_data = c.execute("SELECT hpassword FROM users WHERE email = :email", {"email":email}).fetchone()

        if email_data is None:
            self.ids.welcome_label.text = f'Email not found'
            self.ids.email.text = ''
        else:
            for pass_data in password_data:
                if sha256_crypt.verify(password, pass_data):
                    self.ids.login_button.on_release = current("Home page")
                    pass
                else:
                    self.ids.welcome_label.text = f'Incorrect Password'
                    self.ids.password.text = ''

class Home(Screen):
    pass

class Child(Screen):
    def GetBSerial(self):
        try:
            getDevname = self.config.get('bluetoothsettings', 'HC-05')
            self.recv_stream, self.send_stream = self.get_socket_stream(getDevname)
        except jnius.jnius.JavaException as e:
            self.ids.bluetooth_conn.text = '[b]Not Connected[/b]'
    def tell_blue(self):
        toast("Paired to HC-05")

def get_socket_stream(self, name):
    defaultCharBufferSize = 8192
    try:
        blueAdapt = BluetoothAdapter.getDefaultAdapter()
        if self.rfsocket is not None:
            if self.rfsocket.connected:
                reader = InputStreamReader(self.rfsocket.getInputStream(), getEncode)
                recv_stream = BufferedReader(reader)
                send_stream = self.rfsocket.getOutputStream()
            else:
                self.rfsocket = self.device.createRfcommSocketToServiceRecord(UUID.fromString(getUuid))
                if self.get_port_connect():
                    reader = InputStreamReader(self.rfsocket.getInputStream(), getEncode)
                    recv_stream = BufferedReader(reader, defaultCharBufferSize)
                    send_stream = self.rfsocket.getOutputStream()
        else:
            if blueAdapt is not None:
                if blueAdapt.isEnabled():
                    paired_devices = blueAdapt.getBondedDevices().toArray()
                    self.rfsocket = None
                    for self.device in paired_devices:
                        if self.device.getName() == name:
                            if self.device.bluetoothEnabled:
                                self.rfsocket = self.device.createRfcommSocketToServiceRecord(
                                    UUID.fromString(getUuid))
                                if self.rfsocket is not None:
                                    if self.get_port_connect(): #connect and set the port before creating java objects
                                        reader = InputStreamReader(self.rfsocket.getInputStream(), getEncode)
                                        recv_stream = BufferedReader(reader, defaultCharBufferSize)
                                        send_stream = self.rfsocket.getOutputStream()
                                        break
                else:
                    self.ids.bluetooth_conn.text = '[b]Bluetooth not enabled[/b]'
        if recv_stream is not None and send_stream is not None:
            return recv_stream, send_stream
        else:
            return False, False
    except UnboundLocalError as e:
        return False, False
    except TypeError as e:
        return False, False

def get_port_connect(self):
    try:
        if self.rfsocket.port <= 0:
            self.rfsocket = self.device.createRfcommSocket(1) #set the port explicitly
            if not self.rfsocket.connected:
                self.rfsocket.connect()
        else:
            if not self.rfsocket.connected:
                self.rfsocket.connect()
        if self.rfsocket.connected:
            self.ids.bluetooth_conn.text = '[b]Connected[/b]'
        return True
    except jnius.jnius.JavaException as e:
        self.ids.bluetooth_conn.text = '[b]Cannot connect to socket[/b]'

class ChildData(Screen):
    def GetInput(self):
        i = [27,64] #ASCII escape integer and at sign integer
        pre = bytearray(i)
        cmd = 'Connected\n'.encode('UTF-8')
        #extend bytearray
        pre.extend(cmd)
        self.send_stream.write(pre)
        self.send_stream.flush
        if self.rfsocket.connected and self.recv_stream != None:
            if self.weigh_tme > 0:
                while self.recv_stream.ready != None:
                    try:
                        self.scale_output = self.recv_stream.readLine()
                    except jnius.jnius.JavaException as e:
                        print("JavaException: ", e, self.rfsocket.connected)
                        toast("Java Exception")
                    except ValueError as e:
                        print("Misc error: ", e)
                        toast("Misc error")

                    try:
                        self.show_input(self.scale_output)
                    except ValueError:
                        pass

    def show_input(self, gps_lat, gps_lon, imu_x, imu_y, distance):
        text = ('connected to: ' + self.getDevname)
        text1 = ('Latitude: ' + str(gps_lat))
        text2 = ('Longitude: ' + str(gps_lon))
        text3 = ('IMU X: ' + str(imu_x))
        text4 = ('IMU Y: ' + str(imu_y))
        text5 = ('Distance: ' + str(distance) + ' cm')

        self.ids.bluetooth_conn.text = text
        self.ids.gps_lat_data.text = text1
        self.ids.gps_lon_data.text = text2
        self.ids.imu_x_data.text = text3
        self.ids.imu_y_data.text = text4
        self.ids.distance_data.text = text5

class PlyerData(Screen):
    def show_battery_info(self):
        toast(str(battery.status))

        if int(battery_status) < 20:
            try:
                tts.speak("Battery low please recharge")
            except NotImplementedError:
                toast("Battery Low")

    
    def vibrate(self):
        try:
            vibrator.vibrate(time=4)
        except NotImplementedError:
            toast("Sorry. Vibrate not available")
    
    def speak(self, text_to_read):
        try:
            tts.speak(text_to_read)
        except NotImplementedError:
            toast("Sorry. TTS not available")

class Index(Screen):
    def get_id(self):
        toast(uniqueid.id)

class Facial(Screen):
    def buildbuild(self):
        self.model = tf.keras.models.load_model('siamesemodel.h5', custom_objects={'L1Dist':L1Dist})
    
        self.capture = cv2.VideoCapture(0)
        Clock.schedule_interval(self.update, 1.0/33.0)
    
    def update(self, *args):
        ret, frame = self.capture.read()
        frame = frame[120:120+250, 200:200+250, :]
    
        buf = cv2.flip(frame, 0).tostring()
        img_texture = Texture.create(size=(frame.shape[1], frame.shape[0]), colorfmt = 'bgr')
        img_texture.blit_buffer(buf, colorfmt='bgr', bufferfmt='ubyte')
        self.ids.web_cam.texture = img_texture
    
    def preprocess(self, file_path):
        # Read in image from file path
        byte_img = tf.io.read_file(file_path)
        # Load in the image
        img = tf.io.decode_jpeg(byte_img)
    
        # Preprocessing steps - resizing the image to be 100x100x3
        img = tf.image.resize(img, (100,100))
        # Scale image to be between 0 and 1
        img = img / 255.0
    
        # Return image
        return img
    
    def verify(self, *args):
        detection_threshold = 0.1
        verification_threshold = 0.1
    
        SAVE_PATH = os.path.join('application_data', 'input_image', 'input_image.jpg')
        ret, frame = self.capture.read()
        frame = frame[120:120+250, 200:200+250, :]
        cv2.imwrite(SAVE_PATH, frame)
    
        # Build results array
        results = []
        for image in os.listdir(os.path.join('application_data', 'verification_images')):
            input_img = self.preprocess(os.path.join('application_data', 'input_image', 'input_image.jpg'))
            validation_img = self.preprocess(os.path.join('application_data', 'verification_images', image))
    
            # Make Predictions
            result = self.model.predict(list(np.expand_dims([input_img, validation_img], axis=1)))
            results.append(result)
    
        # Detection Threshold: Metric above which a prediciton is considered positive
        detection = np.sum(np.array(results) > detection_threshold)
    
        # Verification Threshold: Proportion of positive predictions / total positive samples
        verification = detection / len(os.listdir(os.path.join('application_data', 'verification_images')))
        verified = verification > verification_threshold
    
        self.ids.verification_label.text = f'Verified' if verification == True else f'Unverified'
    
        Logger.info(results)
        Logger.info(np.sum(np.array(results)> 0.2))
        Logger.info(np.sum(np.array(results)> 0.4))
        Logger.info(np.sum(np.array(results)> 0.5))
        Logger.info(np.sum(np.array(results)> 0.8))
    
        return results, verified

    file_name = "test.jpg"
    filepath = join(App.get_running_app().user_data_dir, file_name)
    
    def camera_callback(self, filepath):
        if(exists(filepath)):
            toast("Saved")
        else:
            toast("Unable to save")
    
    def take_picture(self):
        camera.take_picture(filename = file_name, on_complete = camera_callback)
    pass

class Mapp(Screen):
    def on_start(self):
        try:
            gps.configure(on_location = self.on_gps_location)
            gps.start()
        except NotImplementedError:
            toast("GPS not accessible")

    def on_gps_location(self, **kwargs):
        latitude_value = kwargs['lat']
        longitude_value = kwargs['lon']
        self.ids.map.lat = latitude_value
        self.ids.map.lon = longitude_value
        print(kwargs)

class WindowManager(ScreenManager):
    pass

class Graphh(Screen):
    def draw_graph(self):
        x = [1, 2, 3, 4, 5]
        y = [5, 12, 6, 9, 15]

        plt.plot(x,y)
        plt.ylabel("Y-Axis")
        plt.xlabel("X-Axis")

class NavigationApp(MDApp):
    def request_android_permissions(self):
        from android.permissions import request_permissions, Permission
        request_permissions([Permission.ACCESS_COARSE_LOCATION, Permission.ACCESS_FINE_LOCATION])

    def build(self):

        #Create a database or connect to one
        if platform not in ["android", "ios"]:
            Window.size = (350, 580)

        conn = sqlite3.connect('navigation.db')
        c = conn.cursor()

        c.execute(""" CREATE TABLE if not exists users(
            email VARCHAR(100) NOT NULL UNIQUE,
            name VARCHAR(50) NOT NULL,
            hpassword VARCHAR(200) NOT NULL)
        """)

        c.execute(""" CREATE TABLE if not exists users(
            email VARCHAR(255) NOT NULL UNIQUE,
            name CHAR(50) NOT NULL,
            hpassword VARCHAR(255) NOT NULL)
        """)

        conn.commit()
        conn.close()

        if platform == "android":
            self.request_android_permissions()

        return Builder.load_file('main.kv')

NavigationApp().run()
