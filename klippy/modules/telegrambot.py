"""
# Install telegram API
#     $pip install python-telegram-bot
#
# Config:
[telegram]
token:
chat_id:
"""
import os, io, uuid
import logging
from telegram.ext import Updater, CommandHandler, MessageHandler, Filters
from telegram import KeyboardButton, ReplyKeyboardMarkup

#--------------------------------------------------------------------------------
_TELEGRAM_LOG_LEVEL = logging.INFO
_TELEGRAM_BOT = None
#--------------------------------------------------------------------------------

class TelegramModule(object):
    bot = updater = None
    state = "initiated"
    def __init__(self, config):
        global _TELEGRAM_BOT
        self.printer = printer = config.get_printer()
        printer.register_event_handler("klippy:ready", self._handle_ready)
        printer.register_event_handler("klippy:connect", self._handle_connect)
        printer.register_event_handler("klippy:shutdown", self._handle_shutdown)
        printer.register_event_handler("klippy:halt", self._handle_shutdown)
        printer.register_event_handler("klippy:disconnect", self._handle_disconnect)
        self.logger = printer.logger.getChild("telegram")
        # self.logger.setLevel(_TELEGRAM_LOG_LEVEL)
        self.gcode = printer.lookup_object('gcode')
        self.toolhead = printer.lookup_object('toolhead')
        printer.try_load_module(config, "virtual_sdcard")
        self.sd = printer.lookup_object('virtual_sdcard')
        self.sd.register_done_cb(self.sd_print_cb)
        self.camera = printer.try_load_module(
            config, "videocam", folder="modules")
        self.name = config.getsection('printer').get(
            'name', default="Klipper printer")
        # Get telegram token key
        self.token = config.get('token')
        self.chat_ids = []
        chat_id = config.getint('chat_id', None)
        if chat_id is None:
            chat_ids = config.get('chat_ids', None)
            if chat_ids is not None:
                self.chat_ids = [ int(_id.strip())
                                  for _id in chat_ids.split(",") ]
        else:
            self.chat_ids = [ chat_id ]
        self.logger.debug("Allowed chat_ids: %s" % self.chat_ids)
        # Run bot
        if _TELEGRAM_BOT is not None:
            _TELEGRAM_BOT.stop()
        self.updater = updater = Updater(self.token)
        # get bot
        self.bot = bot = updater.bot
        _TELEGRAM_BOT = updater
        updater.logger.setLevel(_TELEGRAM_LOG_LEVEL)
        # Get the dispatcher to register handlers
        dispatcher = updater.dispatcher
        dispatcher.logger.setLevel(_TELEGRAM_LOG_LEVEL)
        # Adjust bot logging level
        tglogger = logging.getLogger("telegram.bot")
        tglogger.setLevel(_TELEGRAM_LOG_LEVEL)
        joblogger = logging.getLogger("JobQueue")
        joblogger.setLevel(_TELEGRAM_LOG_LEVEL)
        urllogger = logging.getLogger("telegram.vendor.ptb_urllib3.urllib3.connectionpool")
        urllogger.setLevel(_TELEGRAM_LOG_LEVEL)
        # on different commands - answer in Telegram
        dispatcher.add_handler(CommandHandler(["help", "h", "start"], self.help))
        dispatcher.add_handler(CommandHandler("id", self.get_user_id))
        dispatcher.add_handler(CommandHandler("status", self.get_status))
        dispatcher.add_handler(CommandHandler("stop", self.stop_print))
        dispatcher.add_handler(CommandHandler("kill", self.kill))
        dispatcher.add_handler(CommandHandler("restart", self.restart_printer))
        # Periodic reporting
        dispatcher.add_handler(CommandHandler("set",
            self.set_timer, pass_args=True,
            pass_job_queue=True, pass_chat_data=True))
        dispatcher.add_handler(CommandHandler("unset",
            self.unset_timer, pass_chat_data=True))
        # log all errors
        dispatcher.add_error_handler(self.error)
        # Start the Bot
        updater.start_polling()
        # send welcome message
        for chat_id in self.chat_ids:
            bot.sendMessage(chat_id=chat_id,
                text="Hi,\nBot '%s' at your service" % self.name)
        # Add into printer objects
        self.logger.info("TelegramBot started")
    def __del__(self):
        self.updater.stop()
        self.logger.info("telegram bot stop")

    # ============= Private ==============
    def _handle_shutdown(self):
        self.state = 'shutdown'
        self.__send_message("  Machine is stopped")
    def _handle_disconnect(self):
        self.state = 'disconnect'
    def _handle_ready(self):
        self.state = 'ready'
        self.__send_message("  Machine is ready")
    def _handle_connect(self):
        self.state = 'connect'
    def _handle_halt(self):
        self.state = 'halt'
    gcof = None
    gcostat = "Unknown"
    def sd_print_cb(self, status):
        self.gcostat = status
        if status == 'pause':
            self.gcostat = "paused"
        elif status == 'start':
            self.__send_message("Print started")
            self.gcostat = "printing"
            self.__send_status()
        elif status == 'error':
            self.__send_message("Print failed")
            self.__send_status()
        elif status == 'done':
            self.__send_message("Print finished.")
            self.gcostat = "finished"
            self.__send_status()
        elif status == 'loaded':
            self.gcof = self.sd.get_current_file_name()
            self.gcostat = 'initiated'

    # ============= Reporting ============
    def __send_message(self, msg, chat_id=None):
        if chat_id is not None:
            _ids = [chat_id]
        else:
            _ids = self.chat_ids
        for _id in _ids:
            self.bot.send_message(_id, text=msg)
    def __send_photo(self, chat_id):
        if chat_id is not None:
            chat_ids = [chat_id]
        else:
            chat_ids = self.chat_ids
        for _id in chat_ids:
            pic = self.camera.get_pic()
            if len(pic):
                photo = io.BytesIO(pic)
                # Generate random name (UUID)
                photo.name = "%s.jpg" % str(uuid.uuid4())
                self.bot.send_photo(_id, photo)
    def __send_status(self, chat_id=None):
        if chat_id is not None:
            chat_ids = [chat_id]
        else:
            chat_ids = self.chat_ids
        for _id in chat_ids:
            sd = self.sd
            temperature = self.gcode.get_temp(
                self.printer.get_reactor().monotonic())
            status = ['Firmware state is "%s"' % self.state,
                      '  temperatures : %s' % temperature]
            if self.gcof is None:
                status.append("  Not printing.")
            else:
                if self.gcostat == "finished":
                    progress = 100.
                else:
                    progress = sd.get_progress() * 100.
                status.append("  File '%s':" % os.path.basename(self.gcof))
                status.append('    progress : %.1f%%' % (progress,))
                status.append('    state    : %s' % self.gcostat)
            self.__send_photo(_id)
            self.__send_message("\n".join(status), chat_id=_id)
    @staticmethod
    def __send_buttons(update, msg=""):
        buttons = [
            [KeyboardButton("/status")],
            # [KeyboardButton("/stop")],
            # [KeyboardButton("/kill")],
        ]
        reply_mrk = ReplyKeyboardMarkup(buttons)
        update.message.reply_text(
            msg, reply_markup=reply_mrk)

    # ============= Telegram lib =========
    def start(self, bot, update):
        if update.message.from_user['id'] in self.chat_ids:
            self.__send_buttons(update, "Use buttons for fast communication")

    def help(self, bot, update):
        _id = update.message.from_user['id']
        if _id in self.chat_ids:
            msg = 'Available commands:\n'\
                    "  /id        returns your current id.\n" \
                    "  /h         this help\n" \
                    "  /help      this help\n" \
                    "  /start     open buttons\n" \
                    "  /status    current status of the printer\n" \
                    "  /set <sec> periodic status reporting while printing\n" \
                    "  /unset     disable periodic status reporting\n" \
                    "  /stop      stops ongoing print\n" \
                    "  /kill      call emergency stop\n" \
                    "  /restart   restart firmware\n"
            self.__send_buttons(update, msg)
        elif not self.chat_ids:
            self.get_user_id(bot, update)
    def get_user_id(self, bot, update):
        if not self.chat_ids or \
                update.message.from_user['id'] in self.chat_ids:
            update.message.reply_text(
                'Your chat_id is %s\n'
                'Add id to config to get automatic notifications.' %
                update.message.from_user['id'])
    def get_status(self, bot, update):
        _id = update.message.from_user['id']
        self.__send_status(_id)
    def stop_print(self, bot, update):
        if update.message.from_user['id'] in self.chat_ids:
            self.gcode.push_command_to_queue('M25 P1')
            self.gcode.push_command_to_queue('M1')
            if self.gcostat == "printing":
                update.message.reply_text('print stopped')
            else:
                update.message.reply_text('not printing')
    def kill(self, bot, update):
        if update.message.from_user['id'] in self.chat_ids:
            self.gcode.push_command_to_queue('M112', prio=True)
    def restart_printer(self, bot, update):
        if update.message.from_user['id'] in self.chat_ids:
            self.gcode.push_command_to_queue('M999', prio=True)
    def error(self, bot, update, error):
        """Log Errors caused by Updates."""
        self.logger.warning('Update "%s" caused error "%s"', update, error)
    # Periodic reporting
    def _periodic_status(self, bot, job):
        if self.gcostat == "printing":
            self.__send_status(job.context)
    def set_timer(self, bot, update, args, job_queue, chat_data):
        """Add a job to the queue."""
        if update.message.from_user['id'] not in self.chat_ids:
            return
        chat_id = update.message.chat_id
        try:
            # args[0] should contain the time for the timer in seconds
            timeout = int(args[0])
            if timeout < 0:
                raise ValueError
            # Add job to queue
            job = job_queue.run_repeating(
                self._periodic_status, timeout, context=chat_id)
            chat_data['job'] = job
            update.message.reply_text('Successfully started!')
        except (IndexError, ValueError):
            update.message.reply_text('Usage: /set <seconds>')
    def unset_timer(self, bot, update, chat_data):
        """Remove the job if the user changed their mind."""
        if update.message.from_user['id'] not in self.chat_ids:
            return
        if 'job' not in chat_data:
            update.message.reply_text('Periodic reporting not enabled.')
            return
        job = chat_data['job']
        job.schedule_removal()
        del chat_data['job']
        update.message.reply_text('Successfully stopped!')


def load_config(config):
    return TelegramModule(config)
