import 'dart:convert';
import 'dart:typed_data';

import 'package:bike_app/closepass/incident.dart';
import 'package:bike_app/closepass/uploader.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:http/http.dart' as http;
import 'package:http/testing.dart';

import 'helpers.dart';

Incident sample() => Incident(
      id: '20261125-143012',
      time: DateTime.utc(2026, 11, 25, 14, 30, 12),
      lat: 53.3438,
      lng: -6.2546,
      accuracyM: 6,
      distanceM: 0.82,
      speedKmh: 47,
      photoFiles: const ['photo_1.jpg', 'photo_2.jpg', 'photo_3.jpg'],
      photoOffsetsMs: const [300, 600, 900],
      plateStatus: PlateStatus.found,
      plate: confidentPlate,
    );

void main() {
  test('multipart upload: incident JSON, best image, all photos, LiDAR, token', () async {
    late http.Request req;
    final uploader = IncidentUploader(
      endpoint: Uri.parse('https://example.test/api/close-passes'),
      token: 'secret',
      client: MockClient((r) async {
        req = r;
        return http.Response('{"id": 99, "url": "https://example.test/p/99"}', 201);
      }),
    );
    final photos = [Uint8List.fromList([1, 1, 1]), Uint8List.fromList([2, 2, 2]), Uint8List.fromList([3, 3, 3])];
    final result = await uploader.upload(sample(), photos: photos, lidar: Uint8List.fromList([9, 9]));
    expect(result.remoteId, '99');
    expect(result.url, 'https://example.test/p/99');
    expect(req.method, 'POST');
    expect(req.headers['Authorization'], 'Bearer secret');
    expect(req.headers['content-type'], startsWith('multipart/form-data; boundary='));
    final body = latin1.decode(req.bodyBytes);
    expect(RegExp('name="incident"; filename="incident.json"').hasMatch(body), isTrue);
    expect(body, contains('content-type: application/json'));
    expect(RegExp('name="image"; filename="photo_2.jpg"').hasMatch(body), isTrue, reason: 'best image is photo 2');
    expect(RegExp('name="photos"').allMatches(body).length, 3);
    expect(body, contains('name="lidar"; filename="lidar.bin"'));
    // The JSON part.
    final json = jsonDecode(RegExp(r'(\{"schema".*?\})\r\n--', dotAll: true).firstMatch(body)!.group(1)!) as Map<String, dynamic>;
    expect(json['schema'], 'bikenav.closepass.v1');
    expect(json['plate'], '262-D-12345');
    expect(json['passDistanceM'], 0.82);
    expect(json['passSpeedKmh'], 47);
    expect(json['location'], {'lat': 53.3438, 'lng': -6.2546, 'accuracyM': 6.0});
    expect(json['recordedAt'], '2026-11-25T14:30:12.000Z');
    expect(json['bestPhoto'], 2);
    expect(json['simulated'], isFalse);
  });

  test('no token header without a token; non-JSON success is fine', () async {
    late http.Request req;
    final uploader = IncidentUploader(
      endpoint: Uri.parse('https://example.test/u'),
      client: MockClient((r) async {
        req = r;
        return http.Response('OK', 200);
      }),
    );
    final r = await uploader.upload(sample(), photos: const []);
    expect(req.headers.containsKey('Authorization'), isFalse);
    expect(r.url, isNull);
  });

  test('server errors and no connection give clear messages', () async {
    final failing = IncidentUploader(endpoint: Uri.parse('https://example.test/u'), client: MockClient((_) async => http.Response('Bad plate', 422)));
    expect(() => failing.upload(sample(), photos: const []),
        throwsA(isA<UploadException>().having((e) => e.message, 'message', 'Website replied 422: Bad plate')));
    final offline = IncidentUploader(endpoint: Uri.parse('https://example.test/u'), client: MockClient((_) => throw http.ClientException('down')));
    expect(() => offline.upload(sample(), photos: const []), throwsA(isA<UploadException>()));
  });
}
