import 'dart:convert';

import 'package:bike_app/services/place_search.dart';
import 'package:bike_navigation/bike_navigation.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:http/http.dart' as http;
import 'package:http/testing.dart';

// api.openrouteservice.org was switched off on 24 Nov 2026 (410 Gone).
// These pin the replacement addresses on api.heigit.org.
void main() {
  test('routes are requested from api.heigit.org with the key in the header', () async {
    late http.Request seen;
    final client = MockClient((req) async {
      seen = req;
      return http.Response(jsonEncode(_route), 200);
    });
    final r = await OrsClient(apiKey: 'k1', client: client).route(const LatLng(53.34, -6.25), const LatLng(53.33, -6.26));
    expect(seen.url.toString(), 'https://api.heigit.org/openrouteservice/v2/directions/cycling-regular/geojson');
    expect(seen.headers['Authorization'], 'k1');
    expect(r.line.points, hasLength(2));
  });

  test('place search uses the Pelias address on api.heigit.org', () async {
    late http.Request seen;
    final client = MockClient((req) async {
      seen = req;
      return http.Response(jsonEncode({
        'features': [
          {
            'properties': {'label': 'Trinity College, Dublin'},
            'geometry': {'coordinates': [-6.2546, 53.3438]},
          }
        ]
      }), 200);
    });
    final places = await PlaceSearch('k2', client: client).search('Trinity');
    expect(seen.url.host, 'api.heigit.org');
    expect(seen.url.path, '/pelias/v1/autocomplete');
    expect(seen.url.queryParameters['api_key'], 'k2');
    expect(places.single.label, 'Trinity College, Dublin');
  });

  test('a rejected key gives a clear message', () async {
    final client = MockClient((_) async => http.Response('{"error": "Access to this API has been disallowed"}', 403));
    expect(await OrsClient(apiKey: 'bad', client: client).check(), contains('rejected'));
    expect(PlaceSearch('bad', client: client).search('Trinity'), throwsA(predicate((e) => '$e'.contains('rejected the key'))));
  });

  test('a working key checks out', () async {
    final client = MockClient((_) async => http.Response(jsonEncode(_route), 200));
    expect(await OrsClient(apiKey: 'good', client: client).check(), isNull);
  });
}

final _route = {
  'features': [
    {
      'geometry': {
        'coordinates': [
          [-6.2546, 53.3438],
          [-6.2593, 53.3398],
        ]
      },
      'properties': {
        'segments': [
          {
            'steps': [
              {'type': 11, 'way_points': [0, 0], 'instruction': 'Head south'},
              {'type': 10, 'way_points': [1, 1], 'instruction': 'Arrive'},
            ]
          }
        ],
        'summary': {'duration': 120},
      },
    }
  ]
};
