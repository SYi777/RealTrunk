import configparser
import re


class CleanConfigParser(configparser.ConfigParser):
    def get(self, section, option, **kwargs):
        value = super().get(section, option, **kwargs)
        return re.split(r'\s*[;#]', value)[0].strip()
    
    def getint(self, section, option, **kwargs):
        return int(self.get(section, option, **kwargs))
    
    def getfloat(self, section, option, **kwargs):
        return float(self.get(section, option, **kwargs))


class RobotConfig:
    def __init__(self, file_path):
        self.config = CleanConfigParser()
        self.config.read(file_path, encoding='utf-8')
    
    def __getattr__(self, section_name):
        """Dynamically access configuration sections"""
        if section_name in self.config:
            # Return a Section object that will handle option access
            return Section(self.config, section_name)
        raise AttributeError(f"No section '{section_name}' in configuration")
    
    def __getitem__(self, section_name):
        """Dictionary-style access to sections"""
        if section_name in self.config:
            return Section(self.config, section_name)
        raise KeyError(f"No section '{section_name}' in configuration")


class Section:
    """Helper class to dynamically access options within a section"""
    def __init__(self, config, section_name):
        self._config = config
        self._section = section_name
    
    def __getattr__(self, option_name):
        """Dynamically get an option value with automatic type detection"""
        if self._config.has_option(self._section, option_name):
            # Try to determine the appropriate type
            value = self._config.get(self._section, option_name)
            if value.replace('.', '', 1).isdigit():  # Check if it's a number
                if '.' in value:
                    return float(value)
                return int(value)
            return value
        raise AttributeError(f"No option '{option_name}' in section '{self._section}'")
    
    def __getitem__(self, option_name):
        """Dictionary-style access to options"""
        return self.__getattr__(option_name)
    
    def get(self, option_name, default=None):
        """Safe get with default value"""
        try:
            return self.__getattr__(option_name)
        except AttributeError:
            return default
    
    def getint(self, option_name, default=None):
        """Explicitly get as integer"""
        try:
            return self._config.getint(self._section, option_name)
        except (configparser.NoOptionError, ValueError):
            return default
    
    def getfloat(self, option_name, default=None):
        """Explicitly get as float"""
        try:
            return self._config.getfloat(self._section, option_name)
        except (configparser.NoOptionError, ValueError):
            return default
    
    def getboolean(self, option_name, default=None):
        """Explicitly get as boolean"""
        try:
            return self._config.getboolean(self._section, option_name)
        except (configparser.NoOptionError, ValueError):
            return default


# 使用示例
if __name__ == '__main__':
    robot_config = RobotConfig('./scripts/config.ini')
    
    # Access sections and options dynamically
    print(type(robot_config.Head_Servo.publisher_ip))