import 'dart:convert';

import 'package:bike_app/closepass/plate_reader.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:http/http.dart' as http;
import 'package:http/testing.dart';

import 'helpers.dart';

String reply(Map<String, dynamic> content) => jsonEncode({
      'choices': [
        {
          'message': {'role': 'assistant', 'content': jsonEncode(content), 'refusal': null},
        },
      ],
    });

Map<String, dynamic> answer({bool found = true, String plate = '241-d-12345', double confidence = 0.88}) => {
      'plate_found': found,
      'plate': plate,
      'plate_country': 'IE',
      'confidence': confidence,
      'best_image': 2,
      'vehicle_type': 'car',
      'vehicle_colour': 'silver',
      'vehicle_make': 'Toyota',
      'notes': 'Silver car passing on the right in photos 1-3.',
    };

void main() {
  final photos = [PlatePhoto(tinyPng, mimeType: 'image/png', offsetMs: 300), PlatePhoto(tinyPng, offsetMs: 600)];

  test('request: model, prompt, every photo, strict JSON schema', () async {
    late Map<String, dynamic> sent;
    late http.Request req;
    final reader = PlateReader(
      apiKey: ' sk-abc ',
      model: 'gpt-4.1',
      client: MockClient((r) async {
        req = r;
        sent = jsonDecode(r.body) as Map<String, dynamic>;
        return http.Response(reply(answer()), 200);
      }),
    );
    await reader.read(photos, distanceM: 0.84, speedKmh: 47, time: DateTime.utc(2026, 11, 25, 9));
    expect(req.url.toString(), 'https://api.openai.com/v1/chat/completions');
    expect(req.headers['Authorization'], 'Bearer sk-abc');
    expect(sent['model'], 'gpt-4.1');
    final messages = sent['messages'] as List;
    expect(messages[0]['role'], 'system');
    final system = messages[0]['content'] as String;
    for (final must in ['RIGHT', 'county', 'NEVER invent', '"?"', 'Ignore parked', 'confidence']) {
      expect(system, contains(must));
    }
    final content = messages[1]['content'] as List;
    final text = content.first['text'] as String;
    expect(text, contains('0.84 m'));
    expect(text, contains('47 km/h'));
    expect(text, contains('photo 2 at +0.6 s'));
    final images = content.skip(1).toList();
    expect(images.length, 2);
    expect(images[0]['image_url']['url'], startsWith('data:image/png;base64,'));
    expect(images[1]['image_url']['url'], startsWith('data:image/jpeg;base64,'));
    expect(images[0]['image_url']['detail'], 'high');
    final format = sent['response_format'] as Map;
    expect(format['type'], 'json_schema');
    expect(format['json_schema']['strict'], isTrue);
    final schema = format['json_schema']['schema'] as Map;
    expect(schema['additionalProperties'], isFalse);
    expect((schema['required'] as List).toSet(), (schema['properties'] as Map).keys.toSet(),
        reason: 'strict mode needs every property listed as required');
  });

  test('reply: plate normalised and checked against Irish/UK formats', () {
    final r = PlateReader.parseResponse(reply(answer()), model: 'gpt-4.1');
    expect(r.found, isTrue);
    expect(r.plate, '241-D-12345');
    expect(r.plausibleFormat, isTrue);
    expect(r.confident, isTrue);
    expect(r.bestImage, 2);
    expect(r.vehicleSummary, 'silver Toyota car');
    expect(r.model, 'gpt-4.1');

    final unsure = PlateReader.parseResponse(reply(answer(plate: '24?-D-12345', confidence: 0.9)));
    expect(unsure.confident, isFalse, reason: 'an unreadable character is never confident');
    final low = PlateReader.parseResponse(reply(answer(confidence: 0.3)));
    expect(low.confident, isFalse);
    final none = PlateReader.parseResponse(reply(answer(found: false, plate: 'XYZ')));
    expect(none.found, isFalse);
    expect(none.plate, '');
  });

  test('plate formats', () {
    expect(normalisePlate(' 241 d 12345 '), '241-D-12345');
    expect(normalisePlate('08 - ky - 1234'), '08-KY-1234');
    expect(plausiblePlate('241-D-12345'), isTrue);
    expect(plausiblePlate('08-KY-1234'), isTrue);
    expect(plausiblePlate('152-WW-7'), isTrue);
    expect(plausiblePlate('AB12 CDE'), isTrue); // UK
    expect(plausiblePlate('ABZ 1234'), isTrue); // Northern Ireland
    expect(plausiblePlate('241-QQ-12345'), isFalse); // no such county
    expect(plausiblePlate('2411-D-1'), isFalse);
  });

  test('errors become clear messages', () async {
    Future<String> fail(int status, [String body = '{"error":{"message":"details"}}']) async {
      final reader = PlateReader(apiKey: 'k', model: 'm', client: MockClient((_) async => http.Response(body, status)));
      try {
        await reader.read(photos);
        return 'no error';
      } on PlateReaderException catch (e) {
        return e.message;
      }
    }

    expect(await fail(401), contains('rejected the API key'));
    expect(await fail(404), contains('Change the model'));
    expect(await fail(429), contains('credits'));
    expect(await fail(503), contains('Try again later'));
    expect(await fail(200, '{"choices":[{"message":{"content":null,"refusal":"I cannot help"}}]}'), contains('declined'));
    expect(await fail(200, 'not json'), contains('Unexpected reply'));
    expect(
      () => PlateReader(apiKey: '', model: 'm').read(photos),
      throwsA(isA<PlateReaderException>().having((e) => e.message, 'message', contains('Settings'))),
    );
    final offline = PlateReader(apiKey: 'k', model: 'm', client: MockClient((_) => throw http.ClientException('offline')));
    expect(() => offline.read(photos), throwsA(isA<PlateReaderException>()));
  });

  test('key check calls the model endpoint', () async {
    Uri? asked;
    final ok = PlateReader(apiKey: 'k', model: 'gpt-4.1', client: MockClient((r) async {
      asked = r.url;
      return http.Response('{}', 200);
    }));
    expect(await ok.check(), isNull);
    expect(asked.toString(), 'https://api.openai.com/v1/models/gpt-4.1');
    final bad = PlateReader(apiKey: 'k', model: 'x', client: MockClient((_) async => http.Response('{}', 401)));
    expect(await bad.check(), contains('rejected'));
    expect(await PlateReader(apiKey: '', model: 'x').check(), contains('Enter'));
  });
}
