# coding=utf-8
import os
import sys
import logbook
from logbook import Logger,StreamHandler,FileHandler,TimedRotatingFileHandler
from logbook.more import ColorizedStderrHandler


class ChessLogger:
    LOG_DIR = os.path.join("Log")

    def __init__(self):
        # 日志存放路径
        if not os.path.exists(self.LOG_DIR):
            os.makedirs(self.LOG_DIR)
        # 日志打印到屏幕
        log_std = ColorizedStderrHandler(bubble=True)
        log_std.formatter = self.log_type
        # 日志打印到文件
        log_file = TimedRotatingFileHandler(
            os.path.join(self.LOG_DIR, '%s.log' % 'log'),date_format='%Y-%m-%d-%H', bubble=True, encoding='utf-8')
        log_file.formatter = self.log_type

        self.logger = Logger("chessbot_log")
        logbook.set_datetime_format("local")
        self.logger.handlers = []
        self.logger.handlers.append(log_file)
        self.logger.handlers.append(log_std)

    def log_type(self, record, handler):
        log = "[{date}] [{level}] [{filename}] [{func_name}] [{lineno}] {msg}".format(
            date = record.time,                              # 日志时间
            level = record.level_name,                       # 日志等级
            filename = os.path.split(record.filename)[-1],   # 文件名
            func_name = record.func_name,                    # 函数名
            lineno = record.lineno,                          # 行号
            msg = record.message                             # 日志内容
        )
        return log

    # 以后使用add_attr重写
    def info(self, msg):
        self.logger.info(msg)

