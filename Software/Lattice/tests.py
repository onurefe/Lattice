import unittest
import parser
from parser import ParamType

class TimerTest(unittest.TestCase):
	
class TestStringMethods(unittest.TestCase):
    def setUp(self):
        self.tried = False
        self.succeeded = False
        self.parser = parser.Parser()
        self.params_parsing_result = {}

        self.parser.register_trigger(name="try", callback=self.cb_try,
                                     params={'a': ParamType.INTEGER,
                                             'b': ParamType.FLOAT,
                                             'c': ParamType.LETTER})
        self.parser.register_trigger(name="succ", callback=self.cb_succ,
                                     params={'a': ParamType.FLOAT})

    def cb_try(self, params):
        self.tried = True
        self.params_parsing_result = params

    def cb_succ(self, params):
        self.succeeded = True
        self.params_parsing_result = params

    def test_try(self):
        self.parser.feed_line("try a100 b10.0 cX")
        self.assertTrue(self.tried)
        self.assertEqual(100, self.params_parsing_result['a'])
        self.assertAlmostEqual(10.0, self.params_parsing_result['b'])
        self.assertEqual('X', self.params_parsing_result['c'])
        print(self.params_parsing_result)

    def test_succ(self):
        self.parser.feed_line("succ a50.0")
        self.assertTrue(self.succeeded)
        self.assertAlmostEqual(50.0, self.params_parsing_result['a'])
        print(self.params_parsing_result)

if __name__ == '__main__':
    unittest.main()
