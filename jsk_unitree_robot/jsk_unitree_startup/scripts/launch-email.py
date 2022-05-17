#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from bs4 import BeautifulSoup
import datetime
import json
import random
import subprocess
import emoji
import sys
if sys.version_info.major == 2:
    import urllib2
else:
    from urllib import request


class RobotLaunchEmail:
    """
    Send email when robot launched with tips.
    """
    def __init__(self):
        # Character code differences between python versions
        # See https://stackoverflow.com/questions/54153986/handling-encode-when-converting-from-python2-to-python3 #NOQA
        if sys.version_info.major == 2:
            reload(sys)
            sys.setdefaultencoding('utf-8')
        api_key_file = '/var/lib/robot/openweathermap_api_key.txt'
        with open(api_key_file, 'r') as f:
            self.appid = f.read().split('\n')[0]

    def get_tips(self):
        """
        Get tips from zatsuneta random article.

        Returns:
        ----------
        title : str
            article title
        contents : str
            article contents (heading)
        detail_url :str
            URL of the article written about the details
        """
        url = 'https://zatsuneta.com/category/random.html'
        if sys.version_info.major == 2:
            response = urllib2.urlopen(url)
        else:
            response = request.urlopen(url)
        soup = BeautifulSoup(response, 'html5lib')
        topstories = soup.find('div', class_="article")
        title = topstories.find('a')['title']
        detail_url = topstories.find('a')['href']
        contents = topstories.find('p').text
        response.close()

        return title, contents, detail_url

    # Partly copied from https://github.com/knorth55/jsk_robot/blob/b2999b0cece82d7b1d46320a1e7c84e4fc078bd2/jsk_fetch_robot/jsk_fetch_startup/scripts/time_signal.py #NOQA
    def get_weather_forecast(self, lang='en'):
        url = 'http://api.openweathermap.org/data/2.5/weather?q=tokyo&lang={}&units=metric&appid={}'.format(lang, self.appid)  # NOQA
        if sys.version_info.major == 2:
            resp = json.loads(urllib2.urlopen(url).read())
        else:
            resp = json.loads(request.urlopen(url).read())
        weather = resp['weather'][0]['description']

        forecast_text = ""
        if lang == 'ja':
            forecast_text = "今日の天気は" + weather + "です。"
        else:
            forecast_text = " The weather is " + weather + " now."

        # It feels like the robot is expressing its will based on the weather.
        if "晴" in weather:
            forecast_tuple = (
                '日差しに気をつけて。',
                'お散歩しよう！',
            )
        elif "雨" in weather:
            forecast_tuple = (
                '部屋の中で遊ぼう！',
                '傘忘れていない？',
            )
        elif "雲" or "曇" or "くもり" in weather:
            forecast_tuple = (
                '晴れたらいいな',
            )
        elif "雪" in weather:
            forecast_tuple = (
                '雪合戦しよう！',
                '寒さに気をつけて',
            )
        else:
            forecast_tuple = (' ')
        forecast_text += random.choice(forecast_tuple)

        return forecast_text

    def add_emoji(self, mode):
        """
        0: 普通, 1: 喜び, 2: 安心, 3: 悪巧み, 4: 驚き, 5: 悲しみ, 6: 怒り, 7: 照れ,
        8: 恐怖, 9: 好き, 10: ウインク・おふざけ, 11: 退屈, 12: 混乱
        ref: https://www.webfx.com/tools/emoji-cheat-sheet/
        """
        dic = {0: ":neutral_face:", 1: ":smile:", 2: ":relieved:", 3: ":smirk:" ,
               4: ":astonished:", 5: ":cry:", 6: ":angry:", 7: ":flushed:",
               8: ":scream:", 9: ":heart_eyes:", 10: ":wink:", 11: ":sleepy:", 12: ":sweat:"}

        return emoji.emojize(dic[mode], language='alias')

    def get_fortune(self):
        """
        Get tips from horoscope
        Return:
            message : str
        """
        def add_comment_rank(rank):
            if rank == 1:
                message = "すごい、1位だ" + self.add_emoji(4)
            elif rank <= 3:
                message = str(rank) + "位！いい感じ" + self.add_emoji(1)
            elif rank == 12:
                message = "最下位..." + self.add_emoji(5) + "ラッキーアイテムをチェックしなきゃ！！"
            else:
                message = str(rank) + "位かぁ。そこそこかな" + self.add_emoji(0)
            return "  " + message

        def add_comment_love(point):
            if point >= 9:
                message = "出会いを求めてお散歩しちゃおっかな" + self.add_emoji(9)
            elif point >= 6:
                message = "気になってるあの子に会えちゃうかも" + self.add_emoji(7)
            elif point <= 3:
                message = "こんなの信じないぞ" + self.add_emoji(6)
            else:
                message = "平凡な一日になりそう..." + self.add_emoji(11)
            return "  " + message

        def add_comment_money(point):
            if point >= 9:
                message = "今日はお買い物しちゃおうかな" + self.add_emoji(10)
            elif point <= 3:
                message = "お金のつかい過ぎには気をつけよう..." + self.add_emoji(8)
            else:
                message = "今日は何事もなさそうかな" + self.add_emoji(2)
            return "  " + message

        def add_comment_business(point):
            if point >= 7:
                message = "研究頑張ったら良いことあるかも" + self.add_emoji(1)
            elif point >= 4:
                message = "いいのか悪いのか分からないなぁ" + self.add_emoji(12)
            else:
                message = "今日は研究さぼっちゃおうかな〜" + self.add_emoji(3)
            return "  " + message

        url = 'https://fortune.yahoo.co.jp/12astro/sagittarius'
        if sys.version_info.major == 2:
            response = urllib2.urlopen(url)
        else:
            response = request.urlopen(url)
        soup = BeautifulSoup(response, "html.parser")
        fortune = soup.find('div', id="jumpdtl").find_all('td')
        f_contents = soup.find('div', class_="yftn12a-md48").find_all('dd')[0].contents[0]
        rank = fortune[-5].contents[0].contents[0][0:2]
        point_overall = fortune[-4].contents[0].attrs['alt']
        point_love = fortune[-3].contents[0].attrs['alt']
        point_money = fortune[-2].contents[0].attrs['alt']
        point_business = fortune[-1].contents[0].attrs['alt']
        message = "今日の星座占い：いて座の運勢は【" + rank + "】" + add_comment_rank(int(rank[0])) + "\n"
        message += f_contents + "\n"
        message += "だって！\n"
        message += "\n"
        message += "総合運: " + point_overall + "\n"
        message += "恋愛運: " + point_love + add_comment_love(int(point_love[-2])) + "\n"
        message += "金運:   " + point_money + add_comment_money(int(point_money[-2])) + "\n"
        message += "仕事運: " + point_business + add_comment_business(int(point_business[-2])) + "\n"
        response.close()

        return message

    def send_mail(self):
        """
        Send mail with mailutils
        """
        sender_address = "unitreepro@jsk.imi.i.u-tokyo.ac.jp"
        receiver_address = "unitree@jsk.imi.i.u-tokyo.ac.jp"
        current_time = datetime.datetime.now()
        mail_title = "Unitree Go1 が起動しました"
        message = ""
        message += "おはよう。"
        message += "{}時{}分です。\\n".format(
            current_time.hour, current_time.minute)
        message += "{} \\n".format(self.get_weather_forecast(lang="ja"))
        message += "\\n"

        contents_title, contents, detail_url = self.get_tips()
        message += "今日の豆知識 \\n"
        message += "{}: {} \\n".format(contents_title, contents)
        message += "詳細: {} \\n".format(detail_url)
        message += "\\n"

        message += self.get_fortune()
        message += "\\n"

        # echo -e option is necessary in raspberry pi
        cmd = "echo -e '{}'".format(message)
        cmd += " | /usr/bin/mail -s '{}' -r {} {}".format(
            mail_title, sender_address, receiver_address)
        exit_code = subprocess.call(cmd, shell=True)

if __name__ == '__main__':
    RobotLaunchEmail = RobotLaunchEmail()
    RobotLaunchEmail.send_mail()
