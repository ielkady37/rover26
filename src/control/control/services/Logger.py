#!/usr/bin/env python3
from zope.interface import implementer
from interface.iLoggable import iLoggable
from helpers.LogFileHandler import LogFileHandler
from DTOs.Log import Log
from DTOs.LogSeverity import LogSeverity
from node.LogPublisherNode import LogPublisherNode

@implementer(iLoggable)
class Logger:
    """
    Static class for logging messages to a file and the GUI.
    """

    __log_file_handler = LogFileHandler()
    __log_publisher = LogPublisherNode()

    @staticmethod
    def logToFile(logSeverity: LogSeverity, message: str, component_name: str) -> Log:
        """
        Publishes the log to a local json file and returns a Log object.

        :param logSeverity: The severity of the log.
        :param message: The message to log.
        :param component_name: The name of the component that generated the log.

        :return: A Log object.
        """
        log = Log(logSeverity, message, component_name)
        Logger.__log_file_handler.writeToFile(log)
        return log

    @staticmethod    
    def logToGUI(logSeverity: LogSeverity, message: str, component_name: str) -> Log:
        """
        Publishes the log to the GUI and returns a Log object.

        :param logSeverity: The severity of the log.
        :param message: The message to log.
        :param component_name: The name of the component that generated the log.

        :return: A Log object.
        """
        log = Log(logSeverity, message, component_name)
        Logger.__log_publisher.publish(logSeverity.value, message, component_name)
        return log