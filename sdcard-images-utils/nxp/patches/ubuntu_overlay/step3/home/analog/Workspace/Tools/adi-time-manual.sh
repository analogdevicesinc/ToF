echo "Reference: https://www.cyberciti.biz/faq/howto-set-date-time-from-linux-command-prompt/

To change both the date and time, use the following syntax
==========================================================

$ timedatectl set-time YYYY-MM-DD HH:MM:SS

Where,

HH : An hour.
MM : A minute.
SS : A second, all typed in two-digit form.
YYYY: A four-digit year.
MM : A two-digit month.
DD: A two-digit day of the month.

For example, set the date ’23rd Nov 2015′ and time to ‘8:10:40 am’, enter:

$ timedatectl set-time '2015-11-23 08:10:40'
$ date

How do I set the time zone using timedatectl command?
=====================================================

To see list all available time zones, enter:
$ timedatectl list-timezones
$ timedatectl list-timezones | more
$ timedatectl list-timezones | grep -i asia
$ timedatectl list-timezones | grep America/New

To set the time zone to ‘Asia/Kolkata’, enter:
$ timedatectl set-timezone 'Asia/Kolkata'"
