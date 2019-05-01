from fake_useragent import UserAgent
import requests
import enum

class EthernetITV(object):
    def __init__(self, ip_address="192.168.1.20", pressure_target = 60, user_name='user', password='user'):
        self.ip_address = ip_address
        self.user_name = user_name
        self.password = password
        self.pressure_target = pressure_target
        self.user_agent = UserAgent()

    def __del__(self):
        del self.ip_address
        del self.user_name
        del self.password
        del self.pressure_target
        del self.user_agent
        del self

    def change_pressure_target(self, pressure):
        self.pressure_target = pressure

    def set_pressure(self, pressure):
        base_url = "http://" + self.ip_address + "/"
        login_url = base_url + "login.html"
        param_url = base_url + "setParam.html"
        status_url = base_url + "statusInfo.html"

        login_credentials = {
            'user':self.user_name,
            'password': self.password,
            'submit': 'Submit'
        }
        pressure_params = {
            'sp' : pressure,
            "pu" : 'p',
            'orplc' : 'on',
            'submit' : 'Submit'
        }

        header = {'User-Agent':str(self.user_agent.chrome)}

        with requests.Session() as session:
            login_url = login_url + "?user=" + self.user_name + "&password=" + self.password + "&submit=Submit"
            login_attempt = session.get(login_url, headers=header)

            status_url = base_url + "statusInfo.html"
            status_attempt = session.get(status_url, headers=header)

            param_url = param_url + "?sp=" + str(pressure) + "&pu=p&orplc=on&submit=Submit"
            pressure_change_attempt = session.get(param_url, headers=header)
#            print("== Pressure Change Attempt ==")
#            print(pressure_change_attempt.text)

    def turn_on(self):
        print("EthernetITV turned on.")
        self.set_pressure(self.pressure_target)

    def turn_off(self):
        print("EthernetITV turned off.")
        self.set_pressure(0)

    class STATES(enum.Enum):
        '''
        ' Class Description: Enum class for keeping a list of states for the ethernet_ip
        ' Author: Jacob Taylor Cassady
        '''
        Pressure_ON = 'pressure_on'
        Pressure_OFF = 'pressure_off'

        @staticmethod
        def test_state(suggested_state):
            switcher = {
                EthernetITV.STATES.Pressure_ON.value : 'pressure_on',
                EthernetITV.STATES.Pressure_OFF.value : 'pressure_off'
            }

            return switcher.get(suggested_state, "invalid_ethernet_ip_state")

        @staticmethod
        def confirm_state(suggested_state):
            suggested_state = EthernetITV.STATES.test_state(suggested_state)

            if suggested_state != "invalid_ethernet_ip_state":
                return True
            else:
                return False


if __name__ == "__main__":
    ethernet_ip = EthernetITV()

    ethernet_ip.set_pressure(0)
    _ = raw_input('Pressure updated to 0')

    ethernet_ip.set_pressure(60)
    _ = raw_input('Pressure updated to 60')

    ethernet_ip.set_pressure(0)
    _ = raw_input('Pressure updated to 0')
