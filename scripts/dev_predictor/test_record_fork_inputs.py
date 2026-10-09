import unittest
from record_fork_inputs import entrance, observe_entrances


class EntranceTest(unittest.TestCase):
    def test_trigger_uses_physical_entrances_and_junction_support(self):
        geometry = {'forked_forest.fork_count':4, 'forked_forest.fork_x_min_m':-16.,
                    'forked_forest.fork_length_m':8., 'forked_forest.junction_clearance_radius_m':2.}
        for i, x in enumerate((-16.,-8.,0.,8.)):
            for y in (-1.9,0,1.9):
                self.assertEqual(entrance((x,y,1.5),geometry),i)
        self.assertIsNone(entrance((-14.,0,1.5),geometry))
        self.assertIsNone(entrance((-16.,2.1,1.5),geometry))
        self.assertIsNone(entrance((-15.51,1.99,1.5),geometry))
        # Labels and the preferred arm do not choose the entrance.
        geometry['low_risk'] = 'left'
        self.assertEqual(entrance((-8.,-.2,1.5),geometry),1)

    def test_reached_during_request_is_failed_capture_not_not_reached(self):
        geometry = {'forked_forest.fork_count':4, 'forked_forest.fork_x_min_m':-16.,
                    'forked_forest.fork_length_m':8., 'forked_forest.junction_clearance_radius_m':2.}
        entries = [{'fork_index':i, 'entrance_x_m':x, 'status':'NOT_REACHED'}
                   for i,x in enumerate((-16.,-8.,0.,8.))]
        observe_entrances({'position_m':[-16.,0,1.5]}, geometry, entries, False)
        self.assertEqual(entries[0]['status'], 'REACHED')
        entries[0]['status'] = 'CAPTURE_REQUESTED'
        observe_entrances({'position_m':[-8.,0,1.5]}, geometry, entries, True)
        observe_entrances({'position_m':[-6.,0,1.5]}, geometry, entries, True)
        self.assertEqual(entries[1]['status'], 'CAPTURE_FAILED')
        self.assertEqual(entries[1]['reason'], 'input_service_busy_at_entrance')
        self.assertEqual(entries[1]['trigger']['position_m'], [-8.,0,1.5])
        self.assertEqual(entries[2]['status'], 'NOT_REACHED')

    def test_bypass_is_captured_without_claiming_junction_entry(self):
        geometry = {'forked_forest.fork_count':4, 'forked_forest.fork_x_min_m':-16.,
                    'forked_forest.fork_length_m':8., 'forked_forest.junction_clearance_radius_m':2.}
        entries = [{'fork_index':i, 'entrance_x_m':x, 'status':'NOT_REACHED'}
                   for i,x in enumerate((-16.,-8.,0.,8.))]
        sample={'position_m':[-.49,-7.,1.5]}
        self.assertIsNone(entrance(sample['position_m'],geometry))
        observe_entrances(sample,geometry,entries,False)
        self.assertEqual(entries[2]['status'],'REACHED')
        self.assertEqual(entries[2]['capture_kind'],'junction_bypass_plane')
        self.assertEqual(entries[2]['trigger'],sample)


if __name__ == '__main__':
    unittest.main()
